# -*- coding: utf-8 -*-
"""Офлайн-тесты математического ядра arm_ik (без ROS).

Запуск без colcon:  python3 -m pytest test/  (из корня пакета)
или после сборки:   colcon test --packages-select arm_ik

Важно: все числовые проверки идут через arm_ik._math_compat, а НЕ через
настоящий numpy — иначе в режиме ARM_IK_FORCE_PURE=1 тесты сравнивали бы
pure-python массивы с numpy-объектами и падали бы на импортах. np здесь —
это либо настоящий numpy, либо чистый Python fallback (одинаковый API).

Файловые тесты (URDF-скрипты, config/*.json) из пакета вычищены: дерево
звеньев строится только из TF-сообщений симулятора (см. test_tree_from_tf.py).
"""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_ik._math_compat import np  # тот же слой, что использует ядро


@pytest.fixture(scope="module")
def model():
    """Стартовая шаблонная модель правой руки (заглушка до получения /tf)."""
    from arm_ik.kinematics import make_default_arm
    return make_default_arm("right")


def _close(a, b, atol=1e-9):
    """allclose поверх _math_compat (работает и с pure-python массивами)."""
    d = max(abs(float(x) - float(y)) for x, y in zip(list(a), list(b)))
    return d <= atol


# ---------------------------------------------------------------- базовая FK
def test_fk_rotation_axis_consistency(model):
    from arm_ik.kinematics import fk
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
    # Якобиан по умолчанию берётся в TCP (с учётом model.tcp_offset),
    # поэтому и конечные разности считаем по TCP — как делает IK-петля.
    q = np.array([0.2, -0.4, 0.3, -1.1, 0.5, 0.3, -0.2])
    J = geometric_jacobian(model, q)
    eps = 1e-6
    p0 = np.array([float(v) for v in fk_tcp(model, q)[0]])
    err = 0.0
    for j in range(7):
        qp = [float(v) for v in q]
        qp[j] += eps
        pp = np.array([float(v) for v in fk_tcp(model, np.array(qp))[0]])
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
    """Для случайных допустимых q: IK(FK(q)) достигает той же позы TCP.

    Проверяем ровно тот алгоритм, который использует нода — ik_multi_seed
    (один DLS из единственного старта закономерно попадает в локальные
    минимумы 7-DOF цепи; сиды-альтернативы это лечат).
    """
    from arm_ik.kinematics import fk, ik_multi_seed
    rng_seed = 7
    # диапазон целей в пределах рабочего пространства шаблона: крупные
    # размахи (±1.2 рад) недостижимы точным DLS-допуском и корректно
    # возвращают ok=False (нода на них включает план Б/failed)
    lo = [max(float(v), -0.6) for v in model.lower]
    up = [min(float(v), 0.6) for v in model.upper]
    n_ok = 0
    rnd = random.Random(rng_seed)
    mid = np.array([(l + u) / 2 for l, u in zip(lo, up)])
    seeds = [mid, np.zeros(7),
             np.array([-0.4, 0.3, 0.0, -0.9, 0.0, 0.2, 0.0]),
             np.array([-0.8, 0.5, 0.4, -1.6, 0.3, 0.4, 0.0])]
    for trial in range(8):
        q_tgt = np.array([rnd.uniform(l, u) for l, u in zip(lo, up)])
        T_des, _ = fk(model, q_tgt)
        q_sol, ok, epos, erot = ik_multi_seed(model, T_des, seeds, max_iter=250,
                                               tol_pos=0.005, tol_rot=0.025)
        if ok:
            T_sol, _ = fk(model, q_sol)
            e = [abs(float(v)) for v in (T_sol[:3, 3] - T_des[:3, 3])]
            err_p = math.sqrt(sum(x * x for x in e))
            assert err_p < 0.01, f"IK pos err {err_p}"
            n_ok += 1
    assert n_ok >= 5, f"IK решил только {n_ok}/8 целей"


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


def test_table_guard_blocks_and_clamps():
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
    """Дельта-цель достижима IK — ровно так, как это делает нода:
    multi-seed (текущая поза + типовые сиды, как seed_poses()) с допуском
    set_goal() (tol*0.5). Малые дельты LM сходится к ~3 мм от цели — остаток
    добивает fine-фаза замкнутым контуром; здесь проверяем сам факт решения."""
    from arm_ik.kinematics import fk, delta_to_T, ik_multi_seed, clip_to_limits
    q = np.array([0.1, -0.2, 0.3, -0.9, 0.2, 0.4, -0.1])
    T0, _ = fk(model, q)
    d = np.array([0.03, 0.02, -0.05, 0.05, -0.05, 0.1])
    T1 = delta_to_T(T0, d, base_ref=False)
    seeds = [q, clip_to_limits(np.zeros(7), model, margin=0.1),
             clip_to_limits(np.array([-0.4, 0.3, 0.0, -0.9, 0.0, 0.2, 0.0]), model, 0.05),
             clip_to_limits(np.array([-0.8, 0.5, 0.4, -1.6, 0.3, 0.4, 0.0]), model, 0.05)]
    q_sol, ok, epos, erot = ik_multi_seed(model, T1, seeds, max_iter=250,
                                          tol_pos=0.005, tol_rot=0.025)
    assert ok, f"IK не решилась: pos {epos:.4f} m rot {erot:.4f} rad"
    assert epos < 0.01 and erot < 0.05
    T2, _ = fk(model, q_sol)
    e = [abs(float(a) - float(b)) for a, b in zip(T2[:3, 3], T1[:3, 3])]
    assert math.sqrt(sum(x * x for x in e)) < 0.01


def test_arm_from_dict_roundtrip(model):
    """arm_from_dict — единственный внутренний способ собрать ArmModel
    (его использует tree_from_tf); файловых загрузчиков в пакете нет."""
    from arm_ik.kinematics import arm_from_dict
    data = {"side": model.side, "tip_link": model.tip_link,
            "base_frame": model.base_frame,
            "tcp_offset": [float(v) for v in model.tcp_offset],
            "joints": [{"name": j.name, "motor_id": j.motor_id,
                        "axis": [float(v) for v in j.axis],
                        "origin_xyz": [float(v) for v in j.origin_xyz],
                        "origin_rpy": [float(v) for v in j.origin_rpy],
                        "limits": [float(j.lower), float(j.upper)]}
                       for j in model.joints]}
    m = arm_from_dict(data)
    assert len(m.joints) == 7 and m.side == model.side
    assert m.names == model.names and m.ids == model.ids

 
def test_position_only_ik_reaches_target_position(model):
    from arm_ik.kinematics import fk, ik_dls
    q0 = np.array([0.1, -0.2, 0.3, -0.9, 0.2, 0.4, -0.1])
    T0, _ = fk(model, q0)
    T_goal = T0.copy()
    T_goal[:3, 3] = T_goal[:3, 3] + np.array([0.015, -0.01, 0.02])
    q_sol, ok, _ = ik_dls(
        model, T_goal, q0, max_iter=300, tol_pos=0.003,
        task_mask=[True, True, True, False, False, False])
    assert ok
    T_sol, _ = fk(model, q_sol)
    assert np.linalg.norm(T_sol[:3, 3] - T_goal[:3, 3]) < 0.003


def test_orientation_only_task_mask_converges(model):
    from arm_ik.kinematics import fk, ik_dls, pose_error
    from arm_ik._math_compat import Rotation
    q0 = np.array([0.1, -0.2, 0.3, -0.9, 0.2, 0.4, -0.1])
    T0, _ = fk(model, q0)
    T_goal = T0.copy()
    T_goal[:3, :3] = Rotation.from_rotvec([0.0, 0.0, 0.12]).as_matrix() @ T0[:3, :3]
    q_sol, ok, _ = ik_dls(
        model, T_goal, q0, max_iter=300, tol_rot=0.01,
        task_mask=[False, False, False, True, True, True])
    assert ok
    T_sol, _ = fk(model, q_sol)
    assert np.linalg.norm(pose_error(T_sol, T_goal)[3:]) < 0.01


def test_table_guard_detects_segment_crossing_between_safe_endpoints():
    from arm_ik.kinematics import ArmModel, JointSpec
    from arm_ik.collision import TableGuard

    joints = [
        JointSpec("j1", 1, np.array([0.0, 0.0, 1.0]),
                  np.array([0.0, 0.0, 0.8]), np.zeros(3), -1.0, 1.0),
        JointSpec("j2", 2, np.array([0.0, 0.0, 1.0]),
                  np.array([1.5, 0.0, 0.0]), np.zeros(3), -1.0, 1.0),
    ]
    model = ArmModel("left", joints, "tip", tcp_offset=np.zeros(3))
    guard = TableGuard(h_table=0.9, x0=0.35, x1=1.2,
                       y_half=0.5, margin=0.02)
    # Все концы сегмента вне прямоугольника стола, но само звено проходит над
    # прямоугольником ниже безопасной высоты.
    assert guard.collides_with_table(model, np.zeros(2))


def test_trajectory_rejects_invalid_limits():
    from arm_ik.trajectory import joint_space_profile
    q = np.zeros(2)
    with pytest.raises(ValueError):
        joint_space_profile(q, q, v_max=0.0)
    with pytest.raises(ValueError):
        joint_space_profile(q, q, dt=-0.01)


def test_pure_python_fallback_trajectory_runs():
    import subprocess
    import textwrap

    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    code = textwrap.dedent("""
        import sys
        sys.path.insert(0, %r)
        from arm_ik._math_compat import HAVE_REAL, np
        from arm_ik.trajectory import joint_space_profile
        assert not HAVE_REAL
        q0 = np.zeros(3)
        q1 = np.array([0.5, -0.4, 0.2])
        trajectory = joint_space_profile(q0, q1, v_max=0.8, a_max=0.4, dt=0.02)
        assert len(trajectory) > 1
        assert np.allclose(trajectory[-1], q1, atol=1e-8)
        max_step = np.abs(np.diff(trajectory, axis=0)).max()
        assert max_step <= 0.8 * (0.02 + 1e-9)
    """) % package_root
    env = dict(os.environ)
    env["ARM_IK_FORCE_PURE"] = "1"
    result = subprocess.run([sys.executable, "-c", code], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
