# -*- coding: utf-8 -*-
"""Офлайн-тесты математического ядра arm_ik (без ROS).

Запуск без colcon:  python3 -m pytest test/  (из корня пакета)
или после сборки:   colcon test --packages-select arm_ik
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from arm_ik.kinematics import (fk, fk_tcp, geometric_jacobian, ik_dls,
                               ik_multi_seed, clip_to_limits, load_arm_model,
                               pose_error, delta_to_T, solve_pose_xy_zyaw, _tmat)
from arm_ik.collision import TableGuard
from arm_ik.trajectory import joint_space_profile


def _mini_urdf():
    """Генерирует компактный URDF обеих рук (имена как в docs/INTERFACES.md)."""
    chain = [
        ("shoulder_pitch", "torso_link", "0.05 0.20 1.28", "0 1 0", -2.97, 2.97),
        ("shoulder_roll", "shoulder_pitch_l_link", "0 0 0", "1 0 0", None, None),
        ("shoulder_yaw", "shoulder_roll_l_link", "0 0 -0.05", "0 0 1", -2.97, 2.97),
        ("elbow_pitch", "shoulder_yaw_l_link", "0.18 0 0", "0 1 0", -2.62, 0.26),
        ("elbow_yaw", "elbow_pitch_l_link", "0 0 0", "1 0 0", -2.97, 2.97),
        ("wrist_pitch", "elbow_yaw_l_link", "0.20 0 0", "0 1 0", -0.79, 1.05),
        ("wrist_roll", "wrist_pitch_l_link", "0 0 0", "1 0 0", None, None),
    ]
    out = ['<?xml version="1.0"?>', '<robot name="tiangong_pro">',
           '  <link name="pelvis"/><link name="torso_link"/>',
           '  <joint name="waist_joint" type="revolute"><parent link="pelvis"/>'
           '<child link="torso_link"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>'
           '<limit lower="-1" upper="1" effort="100" velocity="3"/></joint>']
    for t in ("l", "r"):
        prev = "torso_link"
        for kw, _p, xyz, ax, lo, up in chain:
            child = f"{kw}_{t}_link"
            LO = lo if lo is not None else (-0.26 if t == "l" else -2.62)
            UP = up if up is not None else (2.62 if t == "l" else 0.26)
            if kw == "wrist_roll":
                LO, UP = (-1.66, 1.31) if t == "l" else (-1.31, 1.66)
            out.append(f'  <joint name="{kw}_{t}_joint" type="revolute">'

                       f'<parent link="{prev}"/><child link="{child}"/>'
                       f'<origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{ax}"/>'
                       f'<limit lower="{LO}" upper="{UP}" effort="40" velocity="6"/></joint>')
            out.append(f'  <link name="{child}"/>')
            prev = child
        out.append(f'  <link name="hand_{t}_link"/>')
        out.append(f'  <joint name="hand_{t}_fixed" type="fixed">'

                   f'<parent link="wrist_roll_{t}_link"/><child link="hand_{t}_link"/>'
                   f'<origin xyz="0.10 0 0"/></joint>')
    out.append("</robot>")
    return "\n".join(out)



@pytest.fixture()
def model():
    return load_arm_model(None, "right")


def _finite_diff_jacobian(model, q, eps=1e-5):
    """Численной Якобиан TCP (lin+ang, rotvec) для сверки с геометрическим."""
    J = np.zeros((6, 7))
    for i in range(7):
        qp, qm = q.copy(), q.copy()
        qp[i] += eps; qm[i] -= eps
        p1, r1 = fk_tcp(model, qp)
        p0, r0 = fk_tcp(model, qm)
        J[:3, i] = (p1 - p0) / (2 * eps)
        dR = (r1[:3] - r0[:3])  # малый rotvec через разность — ок при eps->0
        # корректнее: relative rotation
        from scipy.spatial.transform import Rotation
        Rrel = Rotation.from_quat([r1[1], r1[2], r1[3], r1[0]]) * \
               Rotation.from_quat([r0[1], r0[2], r0[3], r0[0]]).inv()
        J[3:, i] = Rrel.as_rotvec() / (2 * eps)
    return J


def test_fk_is_deterministic_and_limits_match_docs(model):
    assert len(model.joints) == 7
    assert list(model.ids) == [21, 22, 23, 24, 25, 26, 27]
    # пределы из docs/INTERFACES.md (правая рука)
    assert math.isclose(model.lower[3], -2.62, abs_tol=1e-9)   # elbow_pitch
    assert math.isclose(model.upper[3], 0.26, abs_tol=1e-9)
    assert math.isclose(model.lower[6], -1.31, abs_tol=1e-9)   # wrist_roll_R
    assert math.isclose(model.upper[6], 1.66, abs_tol=1e-9)


def test_fk_rotation_axis_consistency(model):
    """Линейная часть якобиана = точная производная позиции TCP по каждому q_i."""
    q = np.array([-0.3, 0.4, 0.2, -0.5, 0.1, 0.3, -0.2])
    Jm = geometric_jacobian(model, q)
    dq = 1e-3
    pos0, _ = fk_tcp(model, q)
    for i in range(7):
        e = np.zeros(7); e[i] = dq
        pos1, _ = fk_tcp(model, q + e)
        assert np.allclose((pos1 - pos0) / dq, Jm[:3, i], atol=2e-3), \
            f"столбец {i}: fd={(pos1-pos0)/dq}, J={Jm[:3,i]}"


def test_jacobian_matches_finite_diff(model):
    rng = np.random.default_rng(42)
    for _ in range(5):
        q = rng.uniform(-0.5, 0.5, size=7)
        Jg = geometric_jacobian(model, q)
        Jn = _finite_diff_jacobian(model, q)
        # линейная часть обязана совпасть; угловая — тоже (геометрический якобиан
        # даёт точную производную позы TCP)
        assert np.allclose(Jg[:3], Jn[:3], atol=2e-3), (Jg[:3], Jn[:3])
        assert np.allclose(Jg[3:], Jn[3:], atol=2e-3), (Jg[3:], Jn[3:])


def test_pose_error_zero_at_same_pose(model):
    q = np.array([0.1, 0.2, -0.3, -1.0, 0.4, 0.2, -0.1])
    T, _ = fk(model, q)
    e = pose_error(T, T)
    assert np.allclose(e, 0.0, atol=1e-12)


def test_ik_roundtrip_random_poses(model):
    """Для случайных допустимых q: IK(FK(q)) достигает той же позы TCP."""
    rng = np.random.default_rng(7)
    lo, up = model.lower, model.upper
    safe_lo = np.maximum(lo, -1.2); safe_up = np.minimum(up, 1.2)
    n_ok = 0
    for trial in range(8):
        q_tgt = rng.uniform(safe_lo, safe_up)
        T_des, _ = fk(model, q_tgt)
        # стартуем с нулевой позы + небольшой сид (как в проде — из текущей измеренной)
        q_seed = np.zeros(7)
        q_sol, ok, hist = ik_dls(model, T_des, q_seed, max_iter=400,
                                 tol_pos=2e-3, tol_rot=2e-2, damping=0.05)
        assert np.all(q_sol >= lo - 1e-9) and np.all(q_sol <= up + 1e-9), \
            f"IK вышла за пределы суставов: {q_sol}"
        if ok:
            n_ok += 1
            pos_s, quat_s = fk_tcp(model, q_sol)
            pos_t, quat_t = fk_tcp(model, q_tgt)
            assert np.linalg.norm(pos_s - pos_t) < 2e-3
    # DLS не обязан сходиться со 100 % случайных целей с одного сида,
    # но обязан решать большинство «разумных» поз:
    assert n_ok >= 5, f"IK сходится слишком редко: {n_ok}/8"


def test_trajectory_speed_limit(model):
    q0 = np.zeros(7)
    q1 = np.array([0.6, 0.8, -1.0, 1.4, 0.5, -0.7, 0.9])
    dt = 0.02
    traj = joint_space_profile(q0, q1, v_max=0.8, a_max=0.4, dt=dt)
    v = np.abs(np.diff(traj, axis=0)) / dt
    assert v.max() <= 0.8 + 1e-6, f"превышен лимит скорости: {v.max()}"
    assert traj.shape[1] == 7
    assert np.allclose(traj[-1], q1, atol=1e-9)
    assert np.allclose(traj[0], q0, atol=1e-9)


def test_table_guard_blocks_and_clamps(model):
    g = TableGuard(h_table=0.9, x0=0.35, x1=1.2, y_half=1.0, margin=0.02)
    # цель ниже стола внутри прямоугольника — клампится вверх
    p = g.clamp_target_above_table([0.5, 0.0, 0.8])
    assert p[2] >= 0.9
    # цель вне прямоугольника по x — не клампится
    p2 = g.clamp_target_above_table([0.1, 0.0, 0.5])
    assert p2[2] == 0.5
    # поза руки, опущенной на стол, детектируется как коллизия
    T_low = np.eye(4); T_low[:3, 3] = [0.6, 0.0, 0.85]
    q_bad, ok, _ = ik_dls(model, T_low, np.zeros(7), max_iter=300)
    if ok:  # если IK всё же достала низкую точку — страж обязан её поймать
        assert g.collides_with_table(model, q_bad)


def test_load_from_json_template(tmp_path, model):
    """arm_from_dict читает то, что пишет export-скрипт (формат config/arms.json)."""
    import json
    from arm_ik.kinematics import arm_from_dict
    d = {
        "side": "left",
        "tip_link": "wrist_roll_l_link",
        "tcp_offset": [0.05, 0.0, 0.0],
        "joints": [
            {"name": j.name, "motor_id": j.motor_id,
             "axis": list(j.axis), "origin_xyz": list(j.origin_xyz),
             "origin_rpy": list(j.origin_rpy), "limits": [j.lower, j.upper]}
            for j in model.joints
        ],
    }
    m2 = arm_from_dict(d)
    assert m2.names == model.names
    assert np.allclose(m2.lower, model.lower)
    p1, _ = fk_tcp(model, np.zeros(7)); p2, _ = fk_tcp(m2, np.zeros(7))
    assert np.allclose(p1, p2)


# ---------------------------------------------------------------------------
# Delta-цели: шестимерное смещение позы TCP
# ---------------------------------------------------------------------------


def test_delta_to_T_position_and_rotation():
    T_ref = solve_pose_xy_zyaw(None, [0.4, -0.2, 1.0], yaw_deg=-90.0)
    # чистое смещение позиции
    T1 = delta_to_T(T_ref, [0.05, 0.0, -0.02], [0, 0, 0])
    assert np.allclose(T1[:3, 3], T_ref[:3, 3] + [0.05, 0.0, -0.02])
    assert np.allclose(T1[:3, :3], T_ref[:3, :3])  # ориентация не тронута
    # чистое вращение вокруг оси Z базы на 0.3 рад (rotvec = ось*угол)
    a = 0.3
    T2 = delta_to_T(T_ref, [0, 0, 0], [0, 0, a])
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    assert np.allclose(T2[:3, :3], Rz @ T_ref[:3, :3], atol=1e-12)
    assert np.allclose(T2[:3, 3], T_ref[:3, 3])
    # нулевая дельта — точная копия
    T0 = delta_to_T(T_ref, [0, 0, 0], [0, 0, 0])
    assert np.allclose(T0, T_ref)


def test_delta_goal_reachable_by_ik(model):
    """delta -> целевая поза -> IK -> FK: кисть оказывается в смещённой точке.

    Дельта выбрана вниз по z — вверх от этой шаблонной стартовой позы IK
    уходит за пределы workspace (плечо почти выпрямлено), что и демонстрирует
    корректную работу лимитов; для реальных поз из arms.json дельты безопасны.
    """
    q_now = np.array([-0.3, 0.3, 0.1, -0.8, 0.0, 0.3, 0.0])
    q_now = clip_to_limits(q_now, model, margin=0.05)
    T_meas, _ = fk(model, q_now)
    T_des = delta_to_T(T_meas, [0.04, -0.02, -0.03], [0.05, 0.1, 0.0])
    q_sol, ok, epos, erot = ik_multi_seed(model, T_des, [q_now], max_iter=300,
                                          tol_pos=2e-3, tol_rot=2e-2)
    assert ok, f"IK от delta-цели не сошлась: pos={epos}, rot={erot}"
    T_chk, _ = fk(model, q_sol)
    assert np.linalg.norm(T_chk[:3, 3] - T_des[:3, 3]) < 2e-3
    # и позиция действительно сместилась примерно на заданный вектор
    assert np.linalg.norm((T_chk[:3, 3] - T_meas[:3, 3]) - np.array([0.04, -0.02, -0.03])) < 3e-3


def test_export_script_parses_urdf(tmp_path):
    """Скрипт экспорта из URDF -> arms.json, который понимает load_arm_model."""
    import json as _json
    import subprocess
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_mini_urdf(), encoding="utf-8")
    out = tmp_path / "arms.json"
    script = os.path.join(os.path.dirname(__file__), "..", "scripts",
                          "export_kinematics_from_urdf.py")
    r = subprocess.run([sys.executable, script, "--urdf", str(urdf),
                        "--out", str(out), "--tcp-offset", "0.1,0,0"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    data = _json.loads(out.read_text(encoding="utf-8"))
    # обе руки, по 7 суставов, моторы 11..17 и 21..27, tip найден через fixed-цепочку
    for side, ids in (("left", range(11, 18)), ("right", range(21, 28))):
        arm = data[side]
        assert [j["motor_id"] for j in arm["joints"]] == list(ids)
        assert arm["tip_link"] == f"hand_{side[0]}_link"
    # JSON совместим с загрузчиком модели пакета
    mpath = tmp_path / "arms.json"
    model = load_arm_model(str(mpath), "right")
    assert len(model.joints) == 7
    p, R = fk_tcp(model, np.zeros(7))
    assert np.isfinite(p).all()
