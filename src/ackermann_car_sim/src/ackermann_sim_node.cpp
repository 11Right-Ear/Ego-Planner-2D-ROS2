// Ackermann vehicle fake-localization and joint-state simulator.
//
// The node tracks the B-spline trajectory published by the EGO planner using a
// pure-pursuit controller, integrates a bicycle/Ackermann kinematic model, and
// publishes TF plus joint states for the URDF loaded in RViz.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle)
{
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}
}  // namespace

class AckermannSimNode : public rclcpp::Node
{
public:
  AckermannSimNode()
  : Node("ackermann_sim_node")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 0.65142);
    track_ = declare_parameter<double>("track", 0.584);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.164);
    max_speed_ = declare_parameter<double>("max_speed", 0.80);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 0.80);
    max_steer_ = declare_parameter<double>("max_steer", 0.50);
    max_steer_rate_ = declare_parameter<double>("max_steer_rate", 2.5);
    lookahead_min_ = declare_parameter<double>("lookahead_min", 0.65);
    lookahead_gain_ = declare_parameter<double>("lookahead_gain", 0.50);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.35);
    path_timeout_ = declare_parameter<double>("path_timeout", 1.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    optimized_path_topic_ = declare_parameter<std::string>(
      "optimized_path_topic", "/visual_local_trajectory");

    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/current_pose", rclcpp::QoS(10));
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10));

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&AckermannSimNode::initialPoseCallback, this, std::placeholders::_1));
    optimized_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      optimized_path_topic_, rclcpp::QoS(10),
      std::bind(&AckermannSimNode::pathCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishMapToOdom();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&AckermannSimNode::step, this));

    RCLCPP_INFO(
      get_logger(),
      "Ackermann simulator ready (wheelbase=%.3f m, track=%.3f m, wheel_radius=%.3f m). "
      "Use RViz '2D Pose Estimate', then '2D Goal Pose'.",
      wheelbase_, track_, wheel_radius_);
  }

private:
  void publishMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    speed_ = 0.0;
    steering_ = 0.0;
    wheel_rotation_ = 0.0;
    has_initial_pose_ = true;
    has_path_ = false;
    last_path_time_ = now();

    RCLCPP_INFO(
      get_logger(), "Initial pose accepted: x=%.3f y=%.3f yaw=%.3f rad",
      x_, y_, yaw_);
    publishState(now());
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();
    last_path_time_ = now();
    last_closest_index_ = 0;

    if (has_path_ && path_.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Optimized path frame '%s' differs from map frame '%s'.",
        path_.header.frame_id.c_str(), map_frame_.c_str());
    }
  }

  bool computeControl(double & desired_speed, double & desired_steer)
  {
    if (!has_initial_pose_ || !has_path_ || path_.poses.size() < 2) {
      return false;
    }

    const double path_age = (now() - last_path_time_).seconds();
    if (path_age > path_timeout_) {
      return false;
    }

    const auto & poses = path_.poses;
    std::size_t closest_index = 0;
    double closest_distance_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const double dx = poses[i].pose.position.x - x_;
      const double dy = poses[i].pose.position.y - y_;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < closest_distance_sq) {
        closest_distance_sq = distance_sq;
        closest_index = i;
      }
    }
    last_closest_index_ = closest_index;

    const double dist_to_path_end = std::hypot(
      poses.back().pose.position.x - x_, poses.back().pose.position.y - y_);
    if (closest_index + 1 >= poses.size()) {
      const double end_bearing = std::atan2(
        poses.back().pose.position.y - y_, poses.back().pose.position.x - x_);
      const double end_bearing_error = std::fabs(normalizeAngle(end_bearing - yaw_));
      // Stop at the end of an open-loop B-spline, including a small overshoot
      // where the endpoint is already behind the vehicle.
      if (dist_to_path_end < goal_tolerance_ ||
        (dist_to_path_end < 0.80 && end_bearing_error > 1.20))
      {
        desired_speed = 0.0;
        desired_steer = 0.0;
        return true;
      }
    }

    const double lookahead = clamp(
      lookahead_min_ + lookahead_gain_ * std::fabs(speed_),
      lookahead_min_, 1.5);

    std::size_t target_index = closest_index;
    for (std::size_t i = closest_index; i < poses.size(); ++i) {
      const double dx = poses[i].pose.position.x - x_;
      const double dy = poses[i].pose.position.y - y_;
      if (std::hypot(dx, dy) >= lookahead) {
        target_index = i;
        break;
      }
      target_index = i;
    }

    const double target_x = poses[target_index].pose.position.x;
    const double target_y = poses[target_index].pose.position.y;
    const double alpha = normalizeAngle(
      std::atan2(target_y - y_, target_x - x_) - yaw_);
    const double target_distance = std::max(
      std::hypot(target_x - x_, target_y - y_), 0.05);

    const double curvature = clamp(
      2.0 * std::sin(alpha) / target_distance,
      -std::tan(max_steer_) / wheelbase_,
      std::tan(max_steer_) / wheelbase_);
    desired_steer = clamp(std::atan(wheelbase_ * curvature), -max_steer_, max_steer_);

    const double steering_scale = clamp(
      1.0 - 0.60 * std::fabs(desired_steer) / max_steer_, 0.35, 1.0);
    desired_speed = max_speed_ * steering_scale;

    if (std::fabs(alpha) > 1.10) {
      desired_speed = std::min(desired_speed, 0.25);
    }
    if (target_index + 1 >= poses.size() && dist_to_path_end < 1.0) {
      desired_speed = std::min(desired_speed, std::max(0.15, dist_to_path_end));
    }
    return true;
  }

  void step()
  {
    const auto stamp = now();
    const double dt = 0.02;

    double desired_speed = 0.0;
    double desired_steer = steering_;
    if (!computeControl(desired_speed, desired_steer)) {
      desired_speed = 0.0;
      desired_steer = steering_;
    }

    const double max_steer_step = max_steer_rate_ * dt;
    steering_ += clamp(desired_steer - steering_, -max_steer_step, max_steer_step);
    steering_ = clamp(steering_, -max_steer_, max_steer_);

    const double speed_step = max_acceleration_ * dt;
    speed_ += clamp(desired_speed - speed_, -speed_step, speed_step);
    speed_ = clamp(speed_, 0.0, max_speed_);

    x_ += speed_ * std::cos(yaw_) * dt;
    y_ += speed_ * std::sin(yaw_) * dt;
    yaw_ = normalizeAngle(yaw_ + speed_ * std::tan(steering_) / wheelbase_ * dt);

    wheel_rotation_ += speed_ / wheel_radius_ * dt;
    if (wheel_rotation_ > kPi) {
      wheel_rotation_ = std::fmod(wheel_rotation_, 2.0 * kPi);
    }

    publishState(stamp);
  }

  void publishState(const rclcpp::Time & stamp)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = x_;
    pose_msg.pose.position.y = y_;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    current_pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = orientation.x();
    tf_msg.transform.rotation.y = orientation.y();
    tf_msg.transform.rotation.z = orientation.z();
    tf_msg.transform.rotation.w = orientation.w();
    tf_broadcaster_->sendTransform(tf_msg);

    double left_steer = 0.0;
    double right_steer = 0.0;
    if (std::fabs(steering_) > kEpsilon) {
      const double turn_radius = wheelbase_ / std::tan(steering_);
      const double left_radius = turn_radius - track_ / 2.0;
      const double right_radius = turn_radius + track_ / 2.0;
      if (std::fabs(left_radius) > kEpsilon) {
        left_steer = std::atan(wheelbase_ / left_radius);
      }
      if (std::fabs(right_radius) > kEpsilon) {
        right_steer = std::atan(wheelbase_ / right_radius);
      }
      left_steer = clamp(left_steer, -0.62, 0.62);
      right_steer = clamp(right_steer, -0.62, 0.62);
    }

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name = {
      "front_left_steer_joint", "front_right_steer_joint",
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
    joint_msg.position = {
      left_steer, right_steer,
      wheel_rotation_, wheel_rotation_, wheel_rotation_, wheel_rotation_};
    const double wheel_velocity = speed_ / wheel_radius_;
    joint_msg.velocity = {
      0.0, 0.0, wheel_velocity, wheel_velocity, wheel_velocity, wheel_velocity};
    joint_state_pub_->publish(joint_msg);
  }

  double wheelbase_{0.65142};
  double track_{0.584};
  double wheel_radius_{0.164};
  double max_speed_{0.80};
  double max_acceleration_{0.80};
  double max_steer_{0.50};
  double max_steer_rate_{2.5};
  double lookahead_min_{0.65};
  double lookahead_gain_{0.50};
  double goal_tolerance_{0.35};
  double path_timeout_{1.0};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string optimized_path_topic_{"/visual_local_trajectory"};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr optimized_path_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  bool has_initial_pose_{false};
  bool has_path_{false};
  std::size_t last_closest_index_{0};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double speed_{0.0};
  double steering_{0.0};
  double wheel_rotation_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
