# -*- coding: utf-8 -*-
"""Запуск ЕДИНСТВЕННОЙ IK-ноды обеих рук (+ опциональные встроенные демо-цели).

Кинематическая модель строится из /robot_description или urdf_path;
TF2 используется только для преобразования целевых поз. Время симуляции
берётся из /clock при use_sim_time:=true.

    ros2 launch arm_ik arm_ik.launch.py
    ros2 launch arm_ik arm_ik.launch.py h_table:=0.95
    ros2 launch arm_ik arm_ik.launch.py demo_goals:=true goal_x:=0.45 goal_z:=1.0
    ros2 launch arm_ik arm_ik.launch.py demo_delta:=true delta_side:=right delta_x:=0.05

Отдельных демо-нод больше нет: цели публикует сама нода при demo_goals/demo_delta.
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node_exec():
    """Исполняемый единственной ноды. Порядок поиска устойчив к битым сборкам.

    1) standalone-скрипт из ИСХОДНИКОВ этого пакета (bin/run_arm_ik.py);
    2) собранный скрипт в libexec текущего install (run_arm_ik.py);
    3) иначе — имя как есть (launch даст стандартную ошибку с подсказкой).
    Старые wrapper'ы с циклическим импортом (arm_ik_node.py в libexec от прошлых
    сборок) намеренно не используются: у них совпадало имя файла и модуля.
    """
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for cand in ("run_arm_ik.py", "run_arm_ik"):
        src_bin = os.path.join(pkg_root, "bin", cand)
        if os.path.exists(src_bin):
            return src_bin
    try:
        libdir = os.path.join(get_package_prefix("arm_ik"), "lib", "arm_ik")
        for cand in ("run_arm_ik.py", "run_arm_ik"):
            if os.path.exists(os.path.join(libdir, cand)):
                return cand
    except Exception:
        pass
    return "run_arm_ik.py"


def launch_setup(context, *args, **kwargs):
    def b(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    params = {
        "use_sim_time": b("use_sim_time"),
        "base_frame": LaunchConfiguration("base_frame").perform(context),
        "rate_hz": float(LaunchConfiguration("rate_hz").perform(context)),
        "joint_state_timeout": float(LaunchConfiguration("joint_state_timeout").perform(context)),
        "spd_limit": float(LaunchConfiguration("spd_limit").perform(context)),
        "a_max": float(LaunchConfiguration("a_max").perform(context)),
        "h_table": float(LaunchConfiguration("h_table").perform(context)),
        "table_x0": float(LaunchConfiguration("table_x0").perform(context)),
        "table_x1": float(LaunchConfiguration("table_x1").perform(context)),
        "table_y_half": float(LaunchConfiguration("table_y_half").perform(context)),
        "table_margin": float(LaunchConfiguration("table_margin").perform(context)),
        "waypoint_lift": float(LaunchConfiguration("waypoint_lift").perform(context)),
        "n_seeds": int(LaunchConfiguration("n_seeds").perform(context)),
        "ik_rotation_weight": float(LaunchConfiguration("ik_rotation_weight").perform(context)),
        "delta_max_lin": float(LaunchConfiguration("delta_max_lin").perform(context)),
        "delta_max_rot": float(LaunchConfiguration("delta_max_rot").perform(context)),
        "tol_pos": float(LaunchConfiguration("tol_pos").perform(context)),
        "tol_rot": float(LaunchConfiguration("tol_rot").perform(context)),
        "kp": float(LaunchConfiguration("kp").perform(context)),
        "tcp_offset": LaunchConfiguration("tcp_offset").perform(context),
        "urdf_path": LaunchConfiguration("urdf_path").perform(context),
        "kinematics_base_frame": LaunchConfiguration("kinematics_base_frame").perform(context),
        "delta_reference": LaunchConfiguration("delta_reference").perform(context),
        "delta_accumulate": b("delta_accumulate"),
        "delta_rotation_frame": LaunchConfiguration("delta_rotation_frame").perform(context),
        "enable_table_guard": b("table_guard"),
        # встроенные демо-паблишеры (внутри той же ноды)
        "demo_goals": b("demo_goals"),
        "demo_delta": b("demo_delta"),
        "demo_side": LaunchConfiguration("delta_side").perform(context),
        "demo_x": float(LaunchConfiguration("goal_x").perform(context)),
        "demo_y_l": float(LaunchConfiguration("goal_y_left").perform(context)),
        "demo_y_r": float(LaunchConfiguration("goal_y_right").perform(context)),
        "demo_z": float(LaunchConfiguration("goal_z").perform(context)),
        "demo_yaw_deg": float(LaunchConfiguration("goal_yaw_deg").perform(context)),
        "demo_dx": float(LaunchConfiguration("delta_x").perform(context)),
        "demo_dz": float(LaunchConfiguration("delta_z").perform(context)),
        "demo_period": float(LaunchConfiguration("demo_period").perform(context)),
    }

    return [Node(package="arm_ik", executable=_node_exec(), name="arm_ik_node",
                 parameters=[params], output="screen")]


def generate_launch_description():
    d = lambda name, val, desc="": DeclareLaunchArgument(name, default_value=val, description=desc)
    return LaunchDescription([
        d("use_sim_time", "true", "время из /clock (симулятор)"),
        d("base_frame", "waist_yaw_link", "система координат целей и телеметрии"),
        d("rate_hz", "50", "частота контура и генерации траектории"),
        d("joint_state_timeout", "0.5", "таймаут обратной связи /joint_states, с"),
        d("spd_limit", "0.8", "жёсткий лимит скорости моторов рук, рад/с"),
        d("a_max", "0.4", "лимит ускорения траектории, рад/с^2"),
        d("h_table", "0.9", "высота поверхности стола в guard frame, м"),
        d("table_x0", "0.35", "минимальный X стола в guard frame, м"),
        d("table_x1", "1.2", "максимальный X стола в guard frame, м"),
        d("table_y_half", "1.0", "половина ширины стола по Y, м"),
        d("table_margin", "0.02", "запас от стола, м"),
        d("waypoint_lift", "0.25", "подъём промежуточной цели, м"),
        d("n_seeds", "12", "максимум стартовых конфигураций IK"),
        d("ik_rotation_weight", "0.25", "вес ориентации в задаче IK"),
        d("delta_max_lin", "0.35", "максимальная линейная delta, м"),
        d("delta_max_rot", "1.2", "максимальная угловая delta, рад"),
        d("tol_pos", "0.01", "допуск достижения цели по позиции, м"),
        d("tol_rot", "0.05", "допуск по ориентации, рад"),
        d("kp", "0.6", "усиление P-коррекции недолёта ПД-приводов"),
        d("table_guard", "false", "включить защиту от стола после калибровки"),
        d("tcp_offset", "", "legacy: переопределение смещения TCP"),
        d("urdf_path", "", "путь к tiangong_pro_with_hands.urdf (пусто — /robot_description)"),
        d("kinematics_base_frame", "waist_yaw_link", "база URDF-цепочки обеих рук"),
        d("delta_reference", "measured", "опора delta-целей: measured | last_goal"),
        d("delta_accumulate", "true", "суммировать серию delta-смещений"),
        d("delta_rotation_frame", "base", "оси вращения delta: base | tcp"),
        d("demo_goals", "false", "нода сама публикует абсолютные цели (PoseStamped)"),
        d("goal_x", "0.45", "демо: x цели, м"),
        d("goal_y_left", "0.25", "демо: y левой руки, м"),
        d("goal_y_right", "-0.25", "демо: y правой руки, м"),
        d("goal_z", "1.0", "демо: z цели, м"),
        d("goal_yaw_deg", "-90", "демо: кисть смотрит вниз, град"),
        d("demo_delta", "false", "нода сама публикует delta-смещения (TwistStamped)"),
        d("delta_side", "both", "демо delta: left | right | both"),
        d("delta_x", "0.05", "демо delta: смещение вперёд, м"),
        d("delta_z", "0.0", "демо delta: смещение вверх, м"),
        d("demo_period", "8.0", "демо: период публикации целей, с"),
        OpaqueFunction(function=launch_setup),
    ])
