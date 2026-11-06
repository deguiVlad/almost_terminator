"""ros2 launch arm_ik arm_ik.launch.py [rot_weight:=0.0] [tip_link_right:=...]"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {"rot_weight": ("1.0", float), "tol_pos": ("0.01", float), "tol_rot": ("0.05", float),
        "tip_link_left": ("", str), "tip_link_right": ("", str)}


def generate_launch_description():
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, (_, t) in ARGS.items()}
    params["use_sim_time"] = True
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in ARGS.items()]
        + [Node(package="arm_ik", executable="arm_ik_node", parameters=[params], output="screen")])
