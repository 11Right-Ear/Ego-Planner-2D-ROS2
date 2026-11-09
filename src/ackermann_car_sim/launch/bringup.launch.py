from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ackermann_car_sim")
    default_model = PathJoinSubstitution(
        [package_share, "urdf", "ackermann_hunter.urdf.xacro"]
    )
    default_rviz = PathJoinSubstitution(
        [package_share, "config", "ackermann.rviz"]
    )

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Path to the Ackermann vehicle URDF/xacro model.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 configuration file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 together with the planner and simulator.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="ackermann_car_sim",
                executable="ackermann_sim_node",
                name="ackermann_sim_node",
                output="screen",
                parameters=[
                    {
                        "wheelbase": 0.65142,
                        "track": 0.584,
                        "wheel_radius": 0.164,
                        "max_speed": 0.80,
                        "max_steer": 0.50,
                    }
                ],
            ),
            Node(
                package="ego_planner",
                executable="motion_plan",
                name="motion_plan",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
