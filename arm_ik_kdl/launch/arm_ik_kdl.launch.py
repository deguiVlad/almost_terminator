from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_description", default_value=""),
        DeclareLaunchArgument("base_link", default_value="waist_yaw_link"),
        DeclareLaunchArgument("update_rate", default_value="100.0"),
        Node(
            package="arm_ik_kdl",
            executable="arm_ik_kdl_node",
            name="arm_ik_kdl",
            output="screen",
            parameters=[{
                "robot_description": LaunchConfiguration("robot_description"),
                "base_link": LaunchConfiguration("base_link"),
                "update_rate": LaunchConfiguration("update_rate"),
            }],
        ),
    ])
