"""focus.launch.py — вся система фокусировки одним запуском.

ros2 launch focus_tracking focus.launch.py source:=both hand:=right
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("focus_tracking")
    params = os.path.join(pkg, "config", "focus_params.yaml")

    src_arg = DeclareLaunchArgument("source", default_value="object",
                                    description="object | hand | both")
    hand_arg = DeclareLaunchArgument("hand", default_value="right",
                                     description="left | right")

    controller = Node(
        package="focus_tracking",
        executable="focus_controller_node",
        name="focus_controller_node",
        parameters=[params, {"use_sim_time": True,
                             "source": LaunchConfiguration("source")}],
    )

    hand_tracker = Node(
        package="focus_tracking",
        executable="hand_tracker_node",
        name="hand_tracker_node",
        parameters=[params, {"use_sim_time": True,
                             "hand": LaunchConfiguration("hand")}],
    )

    detector = Node(
        package="focus_tracking",
        executable="object_detector_node.py",
        name="object_detector_node",
        parameters=[params, {"use_sim_time": True}],
    )

    return LaunchDescription([src_arg, hand_arg, controller, hand_tracker, detector])
