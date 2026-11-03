# -*- coding: utf-8 -*-
"""Запуск IK-сервера обеих рук + (опционально) демо-целей.

    ros2 launch arm_ik arm_ik.launch.py
    ros2 launch arm_ik arm_ik.launch.py demo:=true h_table:=0.95 goal_x:=0.45
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("arm_ik")
    model_path = LaunchConfiguration("model_path").perform(context)
    if not model_path:
        cand = os.path.join(pkg, "config", "arms.json")
        model_path = cand if os.path.exists(cand) else ""

    params = {
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true",
        "model_path": model_path,
        "rate_hz": float(LaunchConfiguration("rate_hz").perform(context)),
        "spd_limit": float(LaunchConfiguration("spd_limit").perform(context)),
        "h_table": float(LaunchConfiguration("h_table").perform(context)),
        "table_margin": float(LaunchConfiguration("table_margin").perform(context)),
        "tol_pos": float(LaunchConfiguration("tol_pos").perform(context)),
        "tol_rot": float(LaunchConfiguration("tol_rot").perform(context)),
        "kp": float(LaunchConfiguration("kp").perform(context)),
        "tcp_offset": LaunchConfiguration("tcp_offset").perform(context),
        "enable_table_guard":
            LaunchConfiguration("table_guard").perform(context).lower() == "true",
    }

    nodes = [
        Node(package="arm_ik", executable="arm_ik_node", name="arm_ik_node",
             parameters=[params], output="screen"),
    ]
    if LaunchConfiguration("demo").perform(context).lower() == "true":
        nodes.append(Node(
            package="arm_ik", executable="demo_goal_publisher",
            name="demo_goal_publisher",
            parameters=[{
                "use_sim_time": params["use_sim_time"],
                "x": float(LaunchConfiguration("goal_x").perform(context)),
                "z": float(LaunchConfiguration("goal_z").perform(context)),
            }],
            output="screen"))
    return nodes


def generate_launch_description():
    d = lambda name, val, desc="": DeclareLaunchArgument(name, default_value=val, description=desc)
    return LaunchDescription([
        d("use_sim_time", "true", "время из /clock (симулятор)"),
        d("model_path", "", "путь к config/arms.json (пусто — из share пакета)"),
        d("rate_hz", "50", "частота контура и генерации траектории"),
        d("spd_limit", "0.8", "жёсткий лимит скорости моторов рук, рад/с"),
        d("h_table", "0.9", "высота поверхности стола над pelvis, м"),
        d("table_margin", "0.02", "запас от стола, м"),
        d("tol_pos", "0.01", "допуск достижения цели по позиции, м"),
        d("tol_rot", "0.05", "допуск по ориентации, рад"),
        d("kp", "0.6", "усиление P-коррекции недолёта ПД-приводов"),
        d("table_guard", "true", "включить защиту от стола"),
        d("tcp_offset", "", "смещение TCP от frame запястья x,y,z (если пусто — из arms.json)"),
        d("demo", "false", "запустить демо-паблишер целей"),
        d("goal_x", "0.45", "демо: x цели, м"),
        d("goal_z", "1.0", "демо: z цели, м"),
        OpaqueFunction(function=launch_setup),
    ])
