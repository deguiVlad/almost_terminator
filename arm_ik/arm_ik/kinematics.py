# -*- coding: utf-8 -*-
"""Кинематика 7-DOF руки TienKung 2.0 Pro (URDF-кинематическая цепочка).

Цепочка правой руки (зеркально для левой), базовое звено pelvis:

    body_yaw_joint -> shoulder_pitch_{l,r} -> shoulder_roll_{l,r}
    -> shoulder_yaw_{l,r} -> elbow_pitch_{l,r} -> elbow_yaw_{l,r}
    -> wrist_pitch_{l,r} -> wrist_roll_{l,r}

Звенья заданы в канонической KR511/RRRRRRR-форме (как у URDF с осями X/Y/Z):
для каждого сустава — смещение d вдоль Z_prev, длина a вдоль X_prev,
смещение d_y вдоль Y_prev и угол скрутки alpha вокруг X_prev.

!!! ВНИМАНИЕ: числа ниже — шаблон, собранный по топологии URDF. Перед запуском
на реальном роботе/симуляторе переснимите их из `assets/robots/tiangong_pro/`
(или возьмите из `/robot_description` / `/tf_static`) — скрипт
`scripts/export_kinematics_from_urdf.py` делает это автоматически.

Модуль не зависит от ROS: чистый numpy/scipy, легко тестируется офлайн.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Описание кинематической цепочки
# ---------------------------------------------------------------------------


@dataclass
class JointSpec:
    name: str          # имя сустава как в /joint_states и URDF
    motor_id: int      # ID мотора как в /arm/cmd_pos (bodyctrl_msgs)
    axis: np.ndarray   # ось вращения в системе ПРЕДЫДУЩЕго звена (единичный вектор)
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    lower: float = -math.pi
    upper: float = math.pi


@dataclass
class ArmModel:
    """7-звенная цепочка одной руки от pelvis до link запястья."""
    side: str                      # 'left' | 'right'
    joints: list                   # список JointSpec, len == 7
    tip_link: str                  # имя конечного звена (TF frame кисти)
    base_frame: str = "pelvis"     # система координат цели (относительно корпуса)
    # Смещение точки Tool-Center-Point от системы запястья (в метрах, в рамке tip_link).
    tcp_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    @property
    def names(self):
        return [j.name for j in self.joints]

    @property
    def ids(self):
        return [j.motor_id for j in self.joints]

    @property
    def lower(self):
        return np.array([j.lower for j in self.joints])

    @property
    def upper(self):
        return np.array([j.upper for j in self.joints])


def _tmat(xyz, rpy):
    """4x4 матрица трансформации из xyz + rpy (малые углы, XYZ-эксплицитно, как URDF)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def _rot_axis(axis, q):
    return Rotation.from_rotvec(np.asarray(axis, dtype=float) * q).as_matrix()


def chain_transforms(model: ArmModel, q):
    """Возвращает список 4x4 матриц T_i = T(pelvis <- joint_i_frame) после i-го сустава."""
    q = np.asarray(q, dtype=float)
    assert len(q) == len(model.joints), f"нужно {len(model.joints)} значений, получено {len(q)}"
    T = np.eye(4)
    out = []
    for js, qi in zip(model.joints, q):
        T = T @ _tmat(js.origin_xyz, js.origin_rpy)
        R = _rot_axis(js.axis, qi)
        M = np.eye(4)
        M[:3, :3] = R
        T = T @ M
        out.append(T.copy())
    return out


def fk(model: ArmModel, q):
    """Прямая кинематика: возвращает (T_base_tip 4x4, позиции всех суставов 7x3 включая базу)."""
    Ts = chain_transforms(model, q)
    Ttip = Ts[-1] @ _tmat([0, 0, 0], [0, 0, 0])  # система запястья = последнее звено
    pts = [np.array([0.0, 0.0, 0.0])] + [T[:3, 3] for T in Ts]
    return Ttip, np.array(pts)


def fk_tcp(model: ArmModel, q):
    """Поза TCP (кончик кисти) в рамке pelvis: (position 3, quaternion wxyz 4)."""
    Ttip, _ = fk(model, q)
    Ttcp = Ttip @ _tmat(model.tcp_offset, [0, 0, 0])
    pos = Ttcp[:3, 3]
    quat = Rotation.from_matrix(Ttcp[:3, :3]).as_quat()  # xyzw
    return pos, np.array([quat[3], quat[0], quat[1], quat[2]])


def geometric_jacobian(model: ArmModel, q, point=None):
    """Якобиан 6x7 (линейная скорость точки + угловая скорость, оси world/base).

    point — точка, для которой считается линейная часть (по умолчанию — TCP).
    """
    Ts = chain_transforms(model, q)
    if point is None:
        p = (Ts[-1] @ np.append(model.tcp_offset, 1.0))[:3]
    else:
        p = np.asarray(point, dtype=float)
    J = np.zeros((6, len(model.joints)))
    prev = np.eye(4)
    for i, js in enumerate(model.joints):
        Ti = Ts[i]
        # позиция и ось i-го сустава в базовой системе
        z_i = prev @ np.append(js.axis, 0.0)
        z_i = z_i[:3]
        o_i = prev @ np.append(js.origin_xyz, 1.0)
        # применяем origin ПЕРЕД поворотом: позиция сустава = пред-матрица * origin
        Pprev = np.eye(4)
        if i > 0:
            Pprev = Ts[i - 1]
        o_i = (Pprev @ np.append(js.origin_xyz, 1.0))[:3]
        z_i = (Pprev[:3, :3] @ np.asarray(js.axis, dtype=float))
        r = p - o_i
        J[:3, i] = np.cross(z_i, r)
        J[3:, i] = z_i
        prev = Ti
    return J


# ---------------------------------------------------------------------------
# Решатели обратной кинематики
# ---------------------------------------------------------------------------


def pose_error(T_cur44, T_des44):
    """6-вектор ошибки позы: позиция (3) + ориентация через log(R_err) (3)."""
    e_p = T_des44[:3, 3] - T_cur44[:3, 3]
    R_err = T_des44[:3, :3] @ T_cur44[:3, :3].T
    e_r = Rotation.from_matrix(R_err).as_rotvec()
    return np.concatenate([e_p, e_r])


def clip_to_limits(q, model: ArmModel, margin=0.0):
    return np.clip(q, model.lower + margin, model.upper - margin)


def ik_dls(model: ArmModel, T_des44, q_init, max_iter=120, tol_pos=1e-3,
           tol_rot=2e-2, damping=0.05, step_scale=1.0, weight=None):
    """Итеративная IK: Levenberg-Marquardt / Damped Least Squares.

    Учитывает пределы суставов (clip) и приоритетные веса сочленений.
    Возвращает (q, success, err_norm_history).
    """
    q = np.asarray(q_init, dtype=float).copy()
    W = np.eye(7) if weight is None else np.diag(weight)
    hist = []
    for _ in range(max_iter):
        T, _ = fk(model, q)
        e = pose_error(T, T_des44)
        nrm = np.linalg.norm(e)
        hist.append(nrm)
        if nrm < 1e-6 or (np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot):
            return clip_to_limits(q, model), True, hist
        J = geometric_jacobian(model, q)
        # взвешенный DLS: dq = W J^T (J W J^T + lambda^2 I)^{-1} e
        JW = J @ W
        A = JW @ J.T + (damping ** 2) * np.eye(6)
        dq = W @ J.T @ np.linalg.solve(A, e)
        q_new = clip_to_limits(q + step_scale * dq, model, margin=1e-4)
        if np.allclose(q_new, q, atol=1e-12):
            # упёрся в пределы и дальше не идёт — попробуй уменьшить шаг
            q_new = clip_to_limits(q + 0.5 * step_scale * dq, model, margin=1e-4)
            if np.allclose(q_new, q, atol=1e-12):
                return q, False, hist
        q = q_new
    T, _ = fk(model, q)
    e = pose_error(T, T_des44)
    ok = np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot
    return q, ok, hist


def solve_pose_xy_zyaw(model: ArmModel, target_pos, yaw_deg=-90.0, roll_deg=0.0, pitch_deg=0.0):
    """Удобная постановка задачи «взять сверху»: позиция + ориентация кисти с yaw вокруг Z базы."""
    R = Rotation.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(target_pos, dtype=float)
    return T


# ---------------------------------------------------------------------------
# Встроенные модели рук TienKung 2.0 Pro (Шаблон; обновить из URDF!)
# ---------------------------------------------------------------------------

_ARM_LIMITS = {
    # (lower, upper) по документации INTERFACES.md, рад
    "shoulder_pitch": (-2.97, 2.97),
    "shoulder_roll_L": (-0.26, 2.62),
    "shoulder_roll_R": (-2.62, 0.26),
    "shoulder_yaw": (-2.97, 2.97),
    "elbow_pitch": (-2.62, 0.26),
    "elbow_yaw": (-2.97, 2.97),
    "wrist_pitch": (-0.79, 1.05),
    "wrist_roll_L": (-1.66, 1.31),
    "wrist_roll_R": (-1.31, 1.66),
}


def arm_from_dict(d):
    """Собирает ArmModel из словаря (формат config/arms.json, выгруженный из URDF)."""
    joints = []
    for jd in d["joints"]:
        lo, up = jd["limits"]
        joints.append(JointSpec(
            name=jd["name"], motor_id=int(jd["motor_id"]),
            axis=np.array(jd["axis"], dtype=float),
            origin_xyz=np.array(jd["origin_xyz"], dtype=float),
            origin_rpy=np.array(jd["origin_rpy"], dtype=float),
            lower=float(lo), upper=float(up)))
    return ArmModel(
        side=d["side"], joints=joints, tip_link=d["tip_link"],
        base_frame=d.get("base_frame", "pelvis"),
        tcp_offset=np.array(d.get("tcp_offset", [0.0, 0.0, 0.0]), dtype=float))


def load_arm_model(path=None, side="right"):
    """Загружает модель руки из JSON (сгенерирован export_kinematics_from_urdf.py),
    либо возвращает встроенный шаблон."""
    import json
    import os
    if path is None:
        default = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "arms.json")
        if not os.path.exists(default):
            return make_default_arm(side)
        path = default
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    key = "left" if side == "left" else "right"
    return arm_from_dict(data[key])


def make_default_arm(side: str) -> ArmModel:
    """Шаблонная модель руки. Оси и смещения соответствуют типичной сборке
    tiangong_pro URDF: плечо в 3 осях (X,Y,X...), локоть 2 оси, запястье 2 оси.

    ЗАМЕНИТЬ на точные значения из URDF перед боём (см. export-скрипт).
    """
    s = 1.0 if side == "left" else -1.0
    L = side[0].upper()  # 'L'/'R' в именах суставов робота: shoulder_pitch_l_joint
    tag = "l" if side == "left" else "r"
    y_sh = 0.20 * s       # боковое смещение плеча от средней линии (м) — из URDF
    z_sh = 1.28           # высота плеча над pelvis (м) — из URDF
    a_up = 0.18           # длина плеча до локтя
    a_fore = 0.20         # длина предплечья до запястья
    a_wrist = 0.12        # запястье до TCP кончика пальцев ~

    def J(name, mid, axis, xyz, rpy, lim):
        lo, up = lim
        return JointSpec(name=name, motor_id=mid, axis=np.array(axis, dtype=float),
                         origin_xyz=np.array(xyz, dtype=float),
                         origin_rpy=np.array(rpy, dtype=float),
                         lower=lo, upper=up)

    joints = [
        J(f"shoulder_pitch_{tag}_joint", 11 if side == "left" else 21,
          [0, 1, 0], [0.0, y_sh, z_sh], [0, 0, 0], _ARM_LIMITS["shoulder_pitch"]),
        J(f"shoulder_roll_{tag}_joint", 12 if side == "left" else 22,
          [1, 0, 0], [0, 0, 0], [0, 0, 0],
          _ARM_LIMITS["shoulder_roll_L"] if side == "left" else _ARM_LIMITS["shoulder_roll_R"]),
        J(f"shoulder_yaw_{tag}_joint", 13 if side == "left" else 23,
          [0, 0, 1], [a_up, 0, 0], [0, 0, 0], _ARM_LIMITS["shoulder_yaw"]),
        J(f"elbow_pitch_{tag}_joint", 14 if side == "left" else 24,
          [0, 1, 0], [0, 0, 0], [0, 0, 0], _ARM_LIMITS["elbow_pitch"]),
        J(f"elbow_yaw_{tag}_joint", 15 if side == "left" else 25,
          [1, 0, 0], [a_fore, 0, 0], [0, 0, 0], _ARM_LIMITS["elbow_yaw"]),
        J(f"wrist_pitch_{tag}_joint", 16 if side == "left" else 26,
          [0, 1, 0], [0, 0, 0], [0, 0, 0], _ARM_LIMITS["wrist_pitch"]),
        J(f"wrist_roll_{tag}_joint", 17 if side == "left" else 27,
          [1, 0, 0], [a_wrist, 0, 0], [0, 0, 0],
          _ARM_LIMITS["wrist_roll_L"] if side == "left" else _ARM_LIMITS["wrist_roll_R"]),
    ]
    return ArmModel(
        side=side,
        joints=joints,
        tip_link=f"wrist_roll_{tag}_link",
        base_frame="pelvis",
        tcp_offset=np.array([0.05, 0.0, 0.0]),  # кончик пальцев относительно системы запястья
    )
