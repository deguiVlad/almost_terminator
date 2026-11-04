# -*- coding: utf-8 -*-
"""Офлайн-тесты математического ядра arm_ik (без ROS).

Запуск без colcon:  python3 -m pytest test/  (из корня пакета)
или после сборки:   colcon test --packages-select arm_ik

Важно: все числовые проверки идут через arm_ik._math_compat, а НЕ через
настоящий numpy — иначе в режиме ARM_IK_FORCE_PURE=1 тесты сравнивали бы
pure-python массивы с numpy-объектами и падали бы на импортах. np здесь —
это либо настоящий numpy, либо чистый Python fallback (одинаковый API).
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_ik._math_compat import np  # тот же слой, что использует ядро


def _mini_urdf():
    """Генерирует компактный URDF обеих рук (имена как в docs/INTERFACES.md)."""
    chain = [
        ("shoulder_pitch", "torso_link", "0.05 0.20 1.28", "0 1 0", -2.97, 2.97),
        ("shoulder_roll", "shoulder_pitch_l_link", "0 0 0", "1 0 0", None, None),
        ("shoulder_yaw", "shoulder_roll_l_link", "0 0 -0.05", "0 0 1", -2.97, 2.97),
        ("elbow_pitch", "shoulder_yaw_l_link", "0.18 0 0", "0 1 0", -2.62, 0.26),
        ("elbow_yaw", "elbow_pitch_l_link", "0 0 0", "1 0 0", -2.97, 2.97),
        ("wrist_pitch", "elbow_yaw_l_link", "0.20 0 0", "0 1 0", -1.48, 1.48),
        ("wrist_roll", "wrist_pitch_l_link", "0.12 0 0", "1 0 0", -1.66, 1.66),
    ]
    parts = ['<robot name="mini">', '<link name="torso_link"/>']
    prev = "torso_link"
    for i, (nm, ln, xyz, axis, lo, up) in enumerate(chain):
        child = f"{ln if i else nm + '_l'}_link" if False else f"{nm}_l_link"
        lim = ""
        if lo is not None:
            lim = f'<limit lower="{lo}" upper="{up}" effort="100" velocity="0.8"/>'
        parts.append(f'<joint name="{nm}_l_joint" type="revolute">'
                     f'<parent link="{prev}"/><child link="{child}"/>'
                     f'<origin xyz="{xyz}" rpy="0 0 0"/>'
                     f'<axis xyz="{axis}"/>{lim}</joint>')
        parts.append(f'<link name="{child}"/>')
        prev = child
    parts.append('</robot>')
    return "\n".join(parts)


@pytest.fixture(scope="module")
def model():
    from arm_ik.kinematics import load_arm_model
    return load_arm_model("right")  # конфиг шаблона из config/arms_template.json


def _close(a, b, atol=1e-9):
    """allclose поверх _math_compat (работает и с pure-python массивами)."""
    d = max(abs(float(x) - float(y)) for x, y in zip(list(a), list(b)))
    return d <= atol


# ---------------------------------------------------------------- базовая FK
def test_fk_rotation_axis_consistency(model):
    from arm_ik.kinematics import fk, chain_transforms
    q = np.array([0.0] * 7)
    T0, _ = fk(model, q)
    q2 = np.array([0.0] * 7)
    q2[0] = 0.3
    T1, _ = fk(model, q2)
    # ненулевое смещение TCP при повороте первого сустава
    p0 = [float(v) for v in T0[:3, 3]]
    p1 = [float(v) for v in T1[:3, 3]]
    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
    assert dist > 1e-3


def test_jacobian_matches_finite_diff(model):
    from arm_ik.kinematics import fk_tcp, geometric_jacobian
    q = np.array([0.2, -0.4, 0.3, -1.1, 0.5, 0.3, -0.2])
    J = geometric_jacobian(model, q)
    eps = 1e-6
    T0, _ = fk(model, q)
    p0 = [float(v) for v in T0[:3, 3]]
    err = 0.0
    for j in range(7):
        qp = [float(v) for v in q]
        qp[j] += eps
        Tp, _ = fk(model, np.array(qp))
        pp = [float(v) for v in Tp[:3, 3]]
        dp = [(a - b) / eps for a, b in zip(pp, p0)]
        for k in range(3):
            err = max(err, abs(dp[k] - float(J[k][j])))
    assert err < 1e-3, f"Jacobian mismatch {err}"


def test_pose_error_zero_at_same_pose(model):
    from arm_ik.kinematics import fk, pose_error
    q = np.array([0.1, 0.2, -0.3, -1.0, 0.4, 0.2, -0.1])
    T, _ = fk(model, q)
    e = pose_error(T, T)
    assert all(abs(float(v)) < 1e-12 for v in e)


def test_clip_to_limits_respects_bounds(model):
    from arm_ik.kinematics import clip_to_limits
    q = np.array([10.0, -10.0, 5.0, 5.0, -5.0, 9.0, -9.0])
    qc = clip_to_limits(q, model, margin=1e-4)
    for i in range(7):
        assert float(model.lower[i]) - 1e-3 <= float(qc[i]) <= float(model.upper[i]) + 1e-3


def test_ik_roundtrip_random_poses(model):
    """Для случайных допустимых q: IK(FK(q)) достигает той же позы TCP."""
    from arm_ik.kinematics import fk, ik_dls, fk_tcp
    rng_seed = 7
    lo = [max(float(v), -1.2) for v in model.lower]
    up = [min(float(v), 1.2) for v in model.upper]
    n_ok = 0
    rnd = random.Random(rng_seed)
    for trial in range(8):
        q_tgt = np.array([rnd.uniform(l, u) for l, u in zip(lo, up)])
        T_des, _ = fk(model, q_tgt)
        q0 = np.array([(l + u) / 2 for l, u in zip(lo, up)])
        q_sol, ok = ik_dls(model, T_des, q0)
        if ok:
            T_sol, _ = fk(model, q_sol)
            e = [abs(float(v)) for v in (T_sol[:3, 3] - T_des[:3, 3])]
            err_p = math.sqrt(sum(x * x for x in e))
            assert err_p < 0.01, f"IK pos err {err_p}"
            n_ok += 1
    assert n_ok >= 5, f"IK решил только {n_ok}/8 целей"


import random  # noqa: E402  (для теста выше)


def test_trajectory_speed_limit():
    from arm_ik.trajectory import joint_space_profile
    q0 = np.array([0.0] * 7)
    q1 = np.array([0.9, -0.9, 0.8, -1.2, 0.7, -0.6, 0.5])
    prof = joint_space_profile(q0, q1, v_max=0.8, a_max=2.0, dt=0.02)
    vmax = 0.0
    for k in range(1, len(prof)):
        for j in range(7):
            vmax = max(vmax, abs(float(prof[k][j]) - float(prof[k - 1][j])) / 0.02)
    assert vmax <= 0.8 + 1e-6, f"скорость {vmax} превысила лимит 0.8 рад/с"
    assert _close(prof[-1], q1, atol=1e-6)


def test_table_guard_blocks_and_clamps(model):
    from arm_ik.collision import TableGuard
    g = TableGuard(h_table=0.9, margin=0.05)
    T_low = np.eye(4)
    T_low[:3, 3] = [0.4, 0.2, 0.5]      # ниже стола
    assert g.blocks_T(T_low)
    T_safe = np.eye(4)
    T_safe[:3, 3] = [0.4, 0.2, 1.2]     # над столом
    assert not g.blocks_T(T_safe)
    Tc = g.clamp_goal(T_low)
    assert float(Tc[2][3]) >= 0.9 + 0.05 - 1e-9


def test_load_from_json_template(tmp_path):
    from arm_ik.kinematics import load_arm_config
    cfg = {"left": {"joints": [], "base_frame": "pelvis"}}
    p = tmp_path / "arms.json"
    p.write_text('{"left": {"base_frame": "pelvis", "tip_link": "x", "joints": []}}')
    m = load_arm_config(str(p))
    assert "left" in m


def test_delta_to_T_position_and_rotation():
    from arm_ik.kinematics import delta_to_T
    from arm_ik._math_compat import Rotation
    T0 = np.eye(4)
    T0[:3, 3] = [0.4, 0.0, 1.0]
    d = np.array([0.05, -0.02, 0.01, 0.0, 0.0, 0.1])  # dxyz + rotvec_z
    T1 = delta_to_T(T0, d, base_ref=False)
    p = [float(v) for v in T1[:3, 3]]
    assert abs(p[0] - 0.45) < 1e-9 and abs(p[1] + 0.02) < 1e-9 and abs(p[2] - 1.01) < 1e-9
    ang = float(Rotation.from_matrix([[float(v) for v in row] for row in T1[:3, :3]])
                .as_rotvec()[2])
    assert abs(ang - 0.1) < 1e-6


def test_delta_goal_reachable_by_ik(model):
    from arm_ik.kinematics import fk, delta_to_T, ik_multi_seed
    q = np.array([0.1, -0.2, 0.3, -0.9, 0.2, 0.4, -0.1])
    T0, _ = fk(model, q)
    d = np.array([0.03, 0.02, -0.05, 0.05, -0.05, 0.1])
    T1 = delta_to_T(T0, d, base_ref=False)
    q_sol, ok = ik_multi_seed(model, T1, q, n_seeds=3)
    assert ok
    T2, _ = fk(model, q_sol)
    e = [abs(float(a) - float(b)) for a, b in zip(T2[:3, 3], T1[:3, 3])]
    assert math.sqrt(sum(x * x for x in e)) < 0.01


def test_export_script_parses_urdf(tmp_path):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "scripts"))
    import export_kinematics_from_urdf as ex
    urdf = _mini_urdf()
    f = tmp_path / "r.urdf"
    f.write_text(urdf)
    cfg = ex.build_config(str(f))
    assert "left" in cfg and len(cfg["left"]["joints"]) == 7
    j0 = cfg["left"]["joints"][0]
    assert abs(j0["lower"] + 2.97) < 1e-6 and abs(j0["upper"] - 2.97) < 1e-6
