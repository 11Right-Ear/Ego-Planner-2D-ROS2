// Gazebo Ackermann path follower.
//
// The Gazebo physics/ros2_control stack owns the real chassis dynamics. This
// node only converts the EGO planner's B-spline path into a body twist and
// publishes the pose feedback expected by the planner.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
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

class GazeboPathFollower : public rclcpp::Node
{
public:
  GazeboPathFollower()
  : Node("ackermann_gazebo_path_follower")
  {
    max_speed_ = declare_parameter<double>("max_speed", 0.75);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.20);
    lookahead_min_ = declare_parameter<double>("lookahead_min", 0.70);
    lookahead_gain_ = declare_parameter<double>("lookahead_gain", 0.45);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.30);
    path_timeout_ = declare_parameter<double>("path_timeout", 1.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/visual_local_trajectory");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/ackermann_steering_controller/odometry");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/ackermann_steering_controller/reference");

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      command_topic_, rclcpp::QoS(10));
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/current_pose", rclcpp::QoS(10));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&GazeboPathFollower::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&GazeboPathFollower::odomCallback, this, std::placeholders::_1));
    odom_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/ackermann_steering_controller/tf_odometry", rclcpp::QoS(10),
      std::bind(&GazeboPathFollower::odomTfCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishMapToOdom();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GazeboPathFollower::step, this));

    RCLCPP_INFO(
      get_logger(),
      "Gazebo path follower ready: path=%s odom=%s command=%s",
      path_topic_.c_str(), odom_topic_.c_str(), command_topic_.c_str());
  }

private:
  void publishMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();
    last_path_time_ = now();
  }

  void odomTfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if (!msg->transforms.empty()) {
      tf_broadcaster_->sendTransform(msg->transforms);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_orientation_ = q;
    has_odom_ = true;
  }

  bool computeControl(double & linear_velocity, double & angular_velocity)
  {
    if (!has_odom_ || !has_path_ || path_.poses.size() < 2) {
      return false;
    }
    if ((now() - last_path_time_).seconds() > path_timeout_) {
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

    const double end_x = poses.back().pose.position.x;
    const double end_y = poses.back().pose.position.y;
    const double distance_to_end = std::hypot(end_x - x_, end_y - y_);
    if (closest_index + 1 >= poses.size()) {
      const double end_bearing_error = std::fabs(normalizeAngle(
        std::atan2(end_y - y_, end_x - x_) - yaw_));
      if (distance_to_end < goal_tolerance_ ||
        (distance_to_end < 0.80 && end_bearing_error > 1.20))
      {
        return false;
      }
    }

    const double lookahead = clamp(
      lookahead_min_ + lookahead_gain_ * std::fabs(linear_velocity),
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
    double curvature = 2.0 * std::sin(alpha) / target_distance;

    linear_velocity = max_speed_ * clamp(
      1.0 - 0.55 * std::fabs(alpha) / kPi, 0.35, 1.0);
    if (target_index + 1 >= poses.size() && distance_to_end < 1.0) {
      linear_velocity = std::min(
        linear_velocity, std::max(0.12, 0.8 * distance_to_end));
    }

    angular_velocity = linear_velocity * curvature;
    if (std::fabs(angular_velocity) > max_angular_velocity_) {
      const double scale = max_angular_velocity_ / std::fabs(angular_velocity);
      linear_velocity *= scale;
      angular_velocity = std::copysign(max_angular_velocity_, angular_velocity);
    }
    return true;
  }

  void publishCurrentPose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = x_;
    pose_msg.pose.position.y = y_;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = odom_orientation_;
    current_pose_pub_->publish(pose_msg);
  }

  void step()
  {
    const auto stamp = now();
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    if (has_odom_) {
      publishCurrentPose(stamp);
      computeControl(linear_velocity, angular_velocity);
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = stamp;
    command.header.frame_id = "base_footprint";
    command.twist.linear.x = linear_velocity;
    command.twist.angular.z = angular_velocity;
    command_pub_->publish(command);
  }

  double max_speed_{0.75};
  double max_angular_velocity_{1.20};
  double lookahead_min_{0.70};
  double lookahead_gain_{0.45};
  double goal_tolerance_{0.30};
  double path_timeout_{1.0};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string path_topic_{"/visual_local_trajectory"};
  std::string odom_topic_{"/ackermann_steering_controller/odometry"};
  std::string command_topic_{"/ackermann_steering_controller/reference"};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odom_tf_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  bool has_path_{false};
  bool has_odom_{false};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  geometry_msgs::msg::Quaternion odom_orientation_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboPathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
