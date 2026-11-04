# -*- coding: utf-8 -*-
"""Кинематика 7-DOF руки TienKung 2.0 Pro (URDF-кинематическая цепочка).

Цепочка правой руки (зеркально для левой), базовое звено pelvis:

    body_yaw_joint -> shoulder_pitch_{l,r} -> shoulder_roll_{l,r}
    -> shoulder_yaw_{l,r} -> elbow_pitch_{l,r} -> elbow_yaw_{l,r}
    -> wrist_pitch_{l,r} -> wrist_roll_{l,r}

Звенья заданы в канонической KR511/RRRRRRR-форме (как у URDF с осями X/Y/Z):
для каждого сустава — смещение d вдоль Z_prev, длина a вдоль X_prev,
смещение d_y вдоль Y_prev и угол скрутки alpha вокруг X_prev.

!!! ВНИМАНИЕ: встроенные числа — только стартовая заглушка. В рантайме точная
цепочка звеньев строится нодой из топиков симулятора /tf + /tf_static
(см. arm_ik.tree_from_tf); файлы URDF/JSON для этого не нужны.

Модуль не зависит от ROS: чистый numpy/scipy, легко тестируется офлайн.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from arm_ik._math_compat import np
from arm_ik._math_compat import Rotation

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


def fk(model: ArmModel, q, tcp=True):
    """Прямая кинематика: возвращает (T_base_tip 4x4, позиции всех суставов 7x3 включая базу).

    tcp=True (по умолчанию) — поза КОНЧИКА КИСТИ (TCP): кадр последнего звена
    с учётом model.tcp_offset. Именно эта поза используется в pose_error /
    IK-целях, поэтому цель и измерение согласованы по определению.
    tcp=False — чистый кадр link запястья (без смещения TCP).
    """
    Ts = chain_transforms(model, q)
    Ttip = Ts[-1]
    if tcp:
        Ttip = Ttip @ _tmat(model.tcp_offset, [0, 0, 0])
    pts = [np.array([0.0, 0.0, 0.0])] + [T[:3, 3] for T in Ts]
    return Ttip, np.array(pts)


def fk_tcp(model: ArmModel, q):
    """Поза TCP (кончик кисти) в рамке pelvis: (position 3, quaternion wxyz 4)."""
    Ttcp, _ = fk(model, q, tcp=True)
    pos = Ttcp[:3, 3]
    quat = Rotation.from_matrix(Ttcp[:3, :3]).as_quat()  # xyzw
    return pos, np.array([quat[3], quat[0], quat[1], quat[2]])


def joint_frames(model: ArmModel, q):
    """Кадры joints «до» их собственного поворота: F_i = T_{i-1} @ origin_i.

    Для i-го сустава: позиция оси o_i = F_i[:3,3], направление оси
    z_i = F_i[:3,:3] @ axis_i (URDF: origin применяется к parent frame,
    затем вращение вокруг axis в этой же рамке).
    """
    q = np.asarray(q, dtype=float)
    frames = []
    T = np.eye(4)
    for js, qi in zip(model.joints, q):
        F = T @ _tmat(js.origin_xyz, js.origin_rpy)
        frames.append(F.copy())
        M = np.eye(4)
        M[:3, :3] = _rot_axis(js.axis, qi)
        T = F @ M
    return frames


def geometric_jacobian(model: ArmModel, q, point=None):
    """Якобиан 6x7 (линейная скорость точки + угловая скорость, оси base/world).

    point — точка, для которой считается линейная часть (по умолчанию — TCP).
    Реализация каноническая: dp/dq_i = z_i x (p - o_i), dR/dq_i = z_i.
    """
    Ts = chain_transforms(model, q)
    if point is None:
        p = (Ts[-1] @ np.append(model.tcp_offset, 1.0))[:3]
    else:
        p = np.asarray(point, dtype=float)
    Fs = joint_frames(model, q)
    J = np.zeros((6, len(model.joints)))
    for i, js in enumerate(model.joints):
        F = Fs[i]
        o_i = F[:3, 3]
        z_i = F[:3, :3] @ np.asarray(js.axis, dtype=float)
        r = p - o_i
        J[:3, i] = np.cross(z_i, r)
        J[3:, i] = z_i
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
           tol_rot=2e-2, damping=0.05, step_scale=1.0, weight=None,
           barrier=None, null_gain=0.0, q_prefer=None):
    """Итеративная IK: Levenberg-Marquardt / Damped Least Squares.

    Учитывает пределы суставов (clip) и приоритетные веса сочленений.
    Опционально:
      barrier   — callable(q) -> dq(7,), барьерный градиент (защита от стола);
      null_gain/q_prefer — подтяжка в нуль-пространстве к предпочтительной
                  позе (использует 7-ю степень свободы, позу не меняет).
    Возвращает (q, success, err_norm_history).
    """
    q = np.asarray(q_init, dtype=float).copy()
    W = np.eye(len(model.joints)) if weight is None else np.diag(weight)
    hist = []

    def _step(cur_q, damp_cur):
        """Один LM-шаг: возвращает (dq, J, A)."""
        J = geometric_jacobian(model, cur_q)
        JW = J @ W
        A = JW @ J.T + (damp_cur ** 2) * np.eye(6)
        dq = W @ J.T @ np.linalg.solve(A, e_vec)
        if barrier is not None:
            dq = dq + barrier(cur_q)
        if null_gain > 0.0 and q_prefer is not None:
            # нуль-пространственная проекция: не влияет на позу TCP
            Jp_inv = W @ J.T @ np.linalg.inv(A)
            N = np.eye(len(model.joints)) - Jp_inv @ J
            dq = dq + null_gain * (N @ (np.asarray(q_prefer) - cur_q))
        return dq, J, A

    damp = float(damping)
    for _ in range(max_iter):
        T, _ = fk(model, q)
        e_vec = pose_error(T, T_des44)
        nrm = np.linalg.norm(e_vec)
        hist.append(nrm)
        if nrm < 1e-6 or (np.linalg.norm(e_vec[:3]) < tol_pos and np.linalg.norm(e_vec[3:]) < tol_rot):
            return clip_to_limits(q, model), True, hist
        # Levenberg–Marquardt с backtracking: уменьшаем шаг (damping растёт),
        # пока норма ошибки не начнёт убывать. Фиксированный damping из DLS
        # застревал в ~2 мм от цели при жёстком допуске tol_pos=1e-3.
        accepted = False
        d_try = damp
        for _bt in range(8):
            dq, _J, _A = _step(q, d_try)
            alpha = step_scale
            for _ls in range(5):
                q_new = clip_to_limits(q + alpha * dq, model, margin=1e-4)
                if np.allclose(q_new, q, atol=1e-12):
                    break
                Tn, _ = fk(model, q_new)
                nn = np.linalg.norm(pose_error(Tn, T_des44))
                if nn < nrm:
                    q = q_new
                    damp = max(float(damping) * 0.5, d_try * 0.7)  # успех — ослабляем демпфер
                    accepted = True
                    break
                alpha *= 0.5
            if accepted:
                break
            d_try = min(d_try * 2.0, 0.9)
        if not accepted:
            return q, False, hist                     # застряли в локальном минимуме
    T, _ = fk(model, q)
    e = pose_error(T, T_des44)
    ok = np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot
    return q, ok, hist


def ik_multi_seed(model: ArmModel, T_des44, seeds, n_seeds=None, jitter=None, **kw):
    """Перебор стартовых поз (боремся с локальными минимумами DLS).

    seeds — 2D-список/массив стартов; либо один seed (тогда можно передать
    n_seeds>1 — к нему добавятся случайные возмущения jitter рад).
    Берём решение с минимальной ошибкой; при равенстве — ближайшее к seeds[0]
    (обычно текущая измеренная поза => минимум движения).
    Возвращает (q, ok, err_pos, err_rot).
    """
    import random as _rnd
    seeds = [np.asarray(s, dtype=float) for s in seeds]         if len(np.shape(seeds)) == 2 else [np.asarray(seeds, dtype=float)]
    if n_seeds and n_seeds > len(seeds):
        base = seeds[0]
        amp = 0.5 if jitter is None else float(jitter)
        rng = _rnd.Random(0)
        while len(seeds) < n_seeds:
            s = np.array([float(v) for v in base])
            for i in range(len(s)):
                s[i] = min(max(s[i] + rng.uniform(-amp, amp),
                               float(model.lower[i])), float(model.upper[i]))
            seeds.append(s)
    # страховка от локальных минимумов DLS: если ни один сид не сошёлся,
    # запускаем дополнительные попытки с более сильным демпфированием —
    # оно медленнее сходится, но устойчивее к переменной амплитуде шага
    # (типичный случай: малая дельта-цель рядом с текущей позой).
    fallback_damps = kw.pop("fallback_damps", (0.10, 0.15))
    attempts = [(s, dict(kw)) for s in seeds]
    solved_any = False
    for s, k in list(attempts):
        q, ok, _h = ik_dls(model, T_des44, s, **k)
        if ok:
            solved_any = True
            break
    if not solved_any:
        # ни один сид не сошёлся — дополнительные попытки с более сильным
        # демпфированием (устойчивее к локальным минимумам малых дельт)
        for damp in fallback_damps:
            for s in seeds[: max(1, len(seeds) // 2)]:
                k2 = dict(kw)
                k2["damping"] = damp
                k2["max_iter"] = max(int(kw.get("max_iter", 120)), 300)
                attempts.append((s, k2))
    best, best_key = None, None
    for s, k in attempts:
        q, ok, hist = ik_dls(model, T_des44, s, **k)
        T, _ = fk(model, q)
        e = pose_error(T, T_des44)
        key = (not ok, float(np.linalg.norm(e[:3])) * 10 + float(np.linalg.norm(e[3:])),
               float(np.linalg.norm(q - np.asarray(seeds[0]))))
        if best_key is None or key < best_key:
            best, best_key = (q, ok, float(np.linalg.norm(e[:3])),
                              float(np.linalg.norm(e[3:]))), key
    return best


def solve_pose_xy_zyaw(model: ArmModel, target_pos, yaw_deg=-90.0, roll_deg=0.0, pitch_deg=0.0):
    """Удобная постановка задачи «взять сверху»: позиция + ориентация кисти с yaw вокруг Z базы."""
    R = Rotation.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(target_pos, dtype=float)
    return T


def pose_to_T(pos, quat_wxyz):
    """4x4 матрица позы из (позиция 3, кватернион wxyz 4)."""
    pos = np.asarray(pos, dtype=float)
    q = np.asarray(quat_wxyz, dtype=float)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    T[:3, 3] = pos
    return T


def delta_to_T(T_ref, dpos=None, drot=None, base_ref=True, delta6=None):
    """Целевая поза из опорной и 6-мерного смещения (delta goal).

    Аргументы (два стиля вызова):
      delta_to_T(T_ref, dpos, drot)          — dpos(3)/drot(3) отдельно;
      delta_to_T(T_ref, delta6)              — один 6-вектор [dx,dy,dz,rx,ry,rz];
      base_ref=True   (по умолчанию): dpos и drot заданы в системе base_frame
                      (pelvis): p_new = p_ref + dpos; R_new = Rot(drot) @ R_ref.
      base_ref=False  — «локальная» дельта: смещение в осях текущей ориентации
                      кисти, вращение постумножением R_new = R_ref @ Rot(drot).
    """
    if delta6 is not None and dpos is None:
        d6 = np.asarray(delta6, dtype=float)
        dpos, drot = d6[:3], d6[3:]
    if dpos is None:
        dpos = np.zeros(3)
    dpos = np.asarray(dpos, dtype=float)
    # convenience: один 6-вектор [dx,dy,dz,rx,ry,rz] передан как dpos
    if (drot is None or len(np.asarray(drot)) == 0) and len(dpos) == 6:
        drot, dpos = dpos[3:], dpos[:3]
    if drot is None:
        drot = np.zeros(3)
    drot = np.asarray(drot, dtype=float)
    if len(dpos) != 3:
        raise ValueError("dpos должен быть 3-мерным (или передайте 6-вектор delta одним аргументом)")
    if len(drot) != 3:
        raise ValueError("drot должен быть 3-мерным (rotation vector)")
    T = np.array(T_ref, dtype=float).copy()
    R_delta = Rotation.from_rotvec(drot).as_matrix()
    if base_ref:
        T[:3, :3] = R_delta @ T_ref[:3, :3]
        T[:3, 3] = T_ref[:3, 3] + dpos
    else:
        T[:3, :3] = T_ref[:3, :3] @ R_delta
        T[:3, 3] = T_ref[:3, 3] + T_ref[:3, :3] @ dpos
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


def load_arm_config(path):
    """Загружает ВЕСЬ конфиг arms.json как dict {'left': ArmModel, 'right': ArmModel}."""
    import json
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for key in ("left", "right"):
        if key in data:
            out[key] = arm_from_dict(data[key])
    return out


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
    """Возвращает модель руки.

    Основной источник дерева звеньев — топики симулятора /tf + /tf_static
    (см. arm_ik.tree_from_tf); файлы для построения кинематики больше НЕ нужны.
    Здесь остаются только два вспомогательных пути:

      * path=None — встроенная стартовая модель (make_default_arm): используется
        как заглушка до получения TF-дерева и в офлайн-тестах;
      * path=<файл> — явный отладочный JSON в формате arms_template.json
        (только по явному параметру ноды model_path; автопоиска файлов нет).
    """
    import json
    import os
    if path:
        full_path = os.path.abspath(os.path.expanduser(path))
        if not os.path.exists(full_path):
            raise FileNotFoundError(
                f"model_path не найден: {full_path}. По умолчанию дерево звеньев "
                f"берётся из /tf + /tf_static (tree_source:=tf) и файл не требуется.")
        with open(full_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        key = "left" if side == "left" else "right"
        return arm_from_dict(data[key])
    return make_default_arm(side)


def make_default_arm(side: str) -> ArmModel:
    """Стартовая (шаблонная) модель руки: кронштейн->плечо->локоть->запястье.

    При q=0 кисть смотрит вперёд (+X базы), звенья идут вдоль +Z предыдущей рамки.
    Используется как заглушка до получения дерева из /tf + /tf_static и в
    офлайн-тестах; в рантайме нода заменяет её точной цепочкой из TF.
    """
    s = 1.0 if side == "left" else -1.0
    tag = "l" if side == "left" else "r"
    y_sh = 0.20 * s       # боковое смещение плеча от средней линии (м)
    z_sh = 1.28           # высота плеча над pelvis (м)
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
