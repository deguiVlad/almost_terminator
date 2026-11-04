# -*- coding: utf-8 -*-
"""Запуск IK-сервера обеих рук + (опционально) демо-целей.

    ros2 launch arm_ik arm_ik.launch.py
    ros2 launch arm_ik arm_ik.launch.py demo:=true h_table:=0.95 goal_x:=0.45
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node_exec(name):
    """Исполняемый ноды: сначала собранный (lib/arm_ik/*), затем исходники пакета.

    Порядок поиска защищает от СТАРЫХ артефактов сборки в чужих workspace
    (например /nssr_sim/install/arm_ik/lib/arm_ik/arm_ik_node.py с циклическим
    импортом): приоритет у bin/ этого исходного пакета, затем entry-point
    console_scripts (`arm_ik` / `demo_*`), и только потом любой собранный .py.
    """
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # 1) свежий standalone-бинарь из ИСХОДНИКОВ этого пакета
    for cand_name in (name, name.replace(".py", "")):
        src_bin = os.path.join(pkg_root, "bin", cand_name)
        if os.path.exists(src_bin):
            return src_bin
    # 2) собранный entry_point (console_scripts) текущего пакета
    try:
        prefix = get_package_prefix("arm_ik")
        base = name.replace(".py", "")
        for cand in ("arm_ik" if base == "arm_ik_node" else base, base):
            libexec = os.path.join(prefix, "lib", "arm_ik", cand)
            if os.path.exists(libexec) and \
               os.path.realpath(os.path.dirname(libexec)) != os.path.realpath(prefix):
                # не берём старые wrapper'ы из чужого install/, если они там есть
                pass
            if os.path.exists(libexec):
                with open(libexec, "rb") as f:
                    head = f.read(4096)
                if b"from arm_ik.arm_ik_node import main" not in head:  # старый битый wrapper
                    return libexec
    except Exception:
        pass
    return name  # пусть launch даст стандартную ошибку


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
        "delta_reference": LaunchConfiguration("delta_reference").perform(context),
        "delta_accumulate":
            LaunchConfiguration("delta_accumulate").perform(context).lower() == "true",
        "enable_table_guard":
            LaunchConfiguration("table_guard").perform(context).lower() == "true",
    }

    nodes = [
        Node(package="arm_ik", executable=_node_exec("arm_ik_node.py"), name="arm_ik_node",
             parameters=[params], output="screen"),
    ]
    if LaunchConfiguration("demo").perform(context).lower() == "true":
        nodes.append(Node(
            package="arm_ik", executable=_node_exec("demo_goal_publisher.py"),
            name="demo_goal_publisher",
            parameters=[{
                "use_sim_time": params["use_sim_time"],
                "x": float(LaunchConfiguration("goal_x").perform(context)),
                "z": float(LaunchConfiguration("goal_z").perform(context)),
            }],
            output="screen"))
    if LaunchConfiguration("demo_delta").perform(context).lower() == "true":
        nodes.append(Node(
            package="arm_ik", executable=_node_exec("demo_delta_publisher.py"),
            name="demo_delta_publisher",
            parameters=[{
                "use_sim_time": params["use_sim_time"],
                "side": LaunchConfiguration("delta_side").perform(context),
                "dx": float(LaunchConfiguration("delta_x").perform(context)),
                "dz": float(LaunchConfiguration("delta_z").perform(context)),
                "period": float(LaunchConfiguration("delta_period").perform(context)),
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
        d("delta_reference", "measured", "опора delta-целей: measured | last_goal"),
        d("delta_accumulate", "false", "суммировать серию delta-смещений"),
        d("demo", "false", "запустить демо-паблишер абсолютных целей"),
        d("goal_x", "0.45", "демо: x цели, м"),
        d("goal_z", "1.0", "демо: z цели, м"),
        d("demo_delta", "false", "запустить демо delta-смещений (TwistStamped)"),
        d("delta_side", "right", "демо delta: left | right | both"),
        d("delta_x", "0.05", "демо delta: смещение вперёд, м"),
        d("delta_z", "0.0", "демо delta: смещение вверх, м"),
        d("delta_period", "6.0", "демо delta: период публикации, с"),
        OpaqueFunction(function=launch_setup),
    ])
