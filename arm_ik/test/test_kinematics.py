"""Офлайн-тесты кинематики на синтетическом URDF с той же структурой суставов, что у TienKung."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from arm_ik.kinematics import Arm, R_to_quat, axis_angle_R, quat_to_R, rot_error, rpy_to_R  # noqa: E402

LIMITS = {"shoulder_pitch": (-2.97, 2.97), "shoulder_roll_l": (-0.26, 2.62), "shoulder_roll_r": (-2.62, 0.26),
          "shoulder_yaw": (-2.97, 2.97), "elbow_pitch": (-2.62, 0.26), "elbow_yaw": (-2.97, 2.97),
          "wrist_pitch": (-0.79, 1.05), "wrist_roll_l": (-1.66, 1.31), "wrist_roll_r": (-1.31, 1.66)}


def _arm_xml(tag, sy):
    def j(name, parent, child, xyz, axis, rpy="0 0 0"):
        key = name if name in LIMITS else f"{name}_{tag}"
        lo, up = LIMITS[key]
        return (f'<joint name="{name}_{tag}_joint" type="revolute"><parent link="{parent}"/>'
                f'<child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/>'
                f'<limit lower="{lo}" upper="{up}" effort="1" velocity="1"/></joint>')
    links = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_yaw", "wrist_pitch",
             "wrist_roll"]
    out = "".join(f'<link name="{n}_{tag}_link"/>' for n in links)
    out += j("shoulder_pitch", "torso", f"shoulder_pitch_{tag}_link", f"0 {0.2 * sy} 0.4", "0 1 0", "0.1 0 0")
    out += j("shoulder_roll", f"shoulder_pitch_{tag}_link", f"shoulder_roll_{tag}_link", "0 0 0", "1 0 0")
    out += j("shoulder_yaw", f"shoulder_roll_{tag}_link", f"shoulder_yaw_{tag}_link", "0 0 -0.1", "0 0 1")
    out += j("elbow_pitch", f"shoulder_yaw_{tag}_link", f"elbow_pitch_{tag}_link", "0 0 -0.15", "0 1 0")
    out += j("elbow_yaw", f"elbow_pitch_{tag}_link", f"elbow_yaw_{tag}_link", "0 0 -0.1", "0 0 1")
    out += j("wrist_pitch", f"elbow_yaw_{tag}_link", f"wrist_pitch_{tag}_link", "0 0 -0.12", "0 1 0")
    out += j("wrist_roll", f"wrist_pitch_{tag}_link", f"wrist_roll_{tag}_link", "0 0 -0.03", "1 0 0")
    # кисть — фиксированный сустав после запястья, как у URDF с руками Inspire
    out += (f'<link name="hand_{tag}_link"/><joint name="hand_{tag}_fixed" type="fixed">'
            f'<parent link="wrist_roll_{tag}_link"/><child link="hand_{tag}_link"/>'
            f'<origin xyz="0 0 -0.08"/></joint>')
    return out


URDF = ('<robot name="fake"><link name="pelvis"/><link name="torso"/>'
        '<joint name="body_yaw_joint" type="revolute"><parent link="pelvis"/><child link="torso"/>'
        '<origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-2.97" upper="3.14" effort="1" velocity="1"/>'
        '</joint>' + _arm_xml("l", 1) + _arm_xml("r", -1) + "</robot>")


@pytest.fixture(params=["left", "right"])
def arm(request):
    return Arm(URDF, request.param, tcp_offset=(0.0, 0.0, -0.05))


def test_chain_and_ids(arm):
    tag = "l" if arm.side == "left" else "r"
    assert arm.names[0] == f"shoulder_pitch_{tag}_joint" and arm.names[-1] == f"wrist_roll_{tag}_joint"
    assert arm.motor_ids == list(range(11, 18) if arm.side == "left" else range(21, 28))


def test_fk_zero_pose(arm):
    # плечо (0, ±0.2, 0.5), roll 0.1 у плеча наклоняет руку; длина до TCP 0.1+0.15+0.1+0.12+0.03+0.05
    T = arm.fk(np.zeros(7))
    L = 0.55
    sy = 1 if arm.side == "left" else -1
    expected = np.array([0.0, 0.2 * sy, 0.5]) + rpy_to_R(0.1, 0, 0) @ np.array([0, 0, -L])
    assert np.allclose(T[:3, 3], expected, atol=1e-9)


def test_tip_link_override():
    a = Arm(URDF, "right", tip="hand_r_link")
    b = Arm(URDF, "right", tcp_offset=(0, 0, -0.08))
    q = np.array([0.3, -0.4, 0.2, -1.0, 0.5, 0.3, 0.2])
    assert np.allclose(a.fk(q), b.fk(q))


def test_jacobian_matches_finite_differences(arm):
    q = np.array([0.3, 0.4 if arm.side == "left" else -0.4, 0.2, -1.0, 0.5, 0.3, 0.2])
    extra = {"body_yaw_joint": 0.4}
    J = arm.jacobian(q, extra)
    T0 = arm.fk(q, extra)
    h = 1e-6
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = h
        T1 = arm.fk(q + dq, extra)
        assert np.allclose((T1[:3, 3] - T0[:3, 3]) / h, J[:3, i], atol=1e-5)
        assert np.allclose(rot_error(T0[:3, :3], T1[:3, :3]) / h, J[3:, i], atol=1e-5)


def test_ik_roundtrip_random_reachable(arm):
    rng = np.random.default_rng(1)
    fails = 0
    for _ in range(40):
        q_true = rng.uniform(arm.lower, arm.upper)
        extra = {"body_yaw_joint": rng.uniform(-1, 1)}
        T = arm.fk(q_true, extra)
        q_now = np.clip(q_true + rng.normal(0, 0.6, 7), arm.lower, arm.upper)
        ok, q, ep, er = arm.solve(T, q_now, extra)
        if not ok:
            fails += 1
            continue
        assert ep < 0.005 and er < 0.03
        assert np.all(q >= arm.lower + 0.05 - 1e-12) and np.all(q <= arm.upper - 0.05 + 1e-12)
        T_sol = arm.fk(q, extra)
        assert np.linalg.norm(T_sol[:3, 3] - T[:3, 3]) < 0.005
    assert fails <= 2


def test_ik_unreachable_reports_failure(arm):
    T = np.eye(4)
    T[:3, 3] = [3.0, 0.0, 0.5]
    ok, q, ep, _ = arm.solve(T, np.zeros(7))
    assert not ok and ep > 1.0
    assert np.all(q >= arm.lower) and np.all(q <= arm.upper)


def test_position_only(arm):
    q_true = np.array([-0.5, 0.3 if arm.side == "left" else -0.3, 0.1, -1.2, 0.2, 0.1, 0.0])
    T = arm.fk(q_true)
    T[:3, :3] = np.eye(3)    # ориентация, которую рука может и не принять, — игнорируется
    ok, q, ep, _ = arm.solve(T, np.zeros(7), orient="none")
    assert ok and ep < 0.005


def test_targets_at_joint_limits(arm):
    # цели, для которых часть суставов стоит на границе допустимого (предел минус запас)
    rng = np.random.default_rng(3)
    fails = 0
    for _ in range(40):
        q_true = rng.uniform(arm.lower, arm.upper)
        for i in rng.choice(7, 2, replace=False):
            q_true[i] = arm.lower[i] + 0.05 if rng.random() < 0.5 else arm.upper[i] - 0.05
        ok, *_ = arm.solve(arm.fk(q_true), np.clip(q_true + rng.normal(0, 0.6, 7), arm.lower, arm.upper))
        fails += not ok
    assert fails <= 2


def test_secondary_orientation_keeps_position(arm):
    rng = np.random.default_rng(4)
    for _ in range(20):
        q_true = rng.uniform(arm.lower, arm.upper)
        T = arm.fk(q_true)
        # ориентация, которую при этой позиции получить нельзя точно (кисть прямая)
        T[:3, :3] = T[:3, :3] @ axis_angle_R([0, 1, 0], 0.8)
        lock = arm.wrist_lock()
        ok, q, ep, er = arm.solve(T, q_true, orient="secondary", locked=lock)
        if not ok:
            continue
        assert ep < 0.005
        assert all(q[i] == v for i, v in lock.items())
        _, q_none, _, er_none = arm.solve(T, q_true, orient="none", locked=lock)
        assert er <= er_none + 1e-6


def test_secondary_reaches_full_pose_when_possible(arm):
    rng = np.random.default_rng(6)
    hits = 0
    for _ in range(20):
        q_true = rng.uniform(arm.lower, arm.upper)
        ok, _, ep, er = arm.solve(arm.fk(q_true), np.clip(q_true + rng.normal(0, 0.4, 7), arm.lower, arm.upper),
                                  orient="secondary")
        hits += ok and er < 0.03
    assert hits >= 17


def test_wrist_lock(arm):
    lock = arm.wrist_lock()
    assert sorted(lock) == [5, 6] and all(v == 0.0 for v in lock.values())


def test_rotation_helpers():
    rng = np.random.default_rng(2)
    for _ in range(50):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        ang = rng.uniform(0, math.pi)
        R = axis_angle_R(axis, ang)
        assert np.allclose(quat_to_R(*R_to_quat(R)), R, atol=1e-9)
        assert np.allclose(rot_error(np.eye(3), R), axis * ang, atol=1e-6)
    R = axis_angle_R([0, 1, 0], math.pi)
    assert np.isclose(np.linalg.norm(rot_error(np.eye(3), R)), math.pi, atol=1e-6)


def test_missing_chain_raises():
    with pytest.raises(ValueError):
        Arm(URDF, "left", tip="no_such_link")
