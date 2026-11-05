# -*- coding: utf-8 -*-
"""Regression tests for the table-plane guard."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_ik._math_compat import np
from arm_ik.collision import TableGuard
from arm_ik.kinematics import make_default_arm


def test_table_guard_clamps_tcp_with_link_radius():
    guard = TableGuard(h_table=0.9, x0=0.3, x1=1.2,
                       y_half=0.6, margin=0.02)
    result = guard.clamp_target_above_table([0.7, 0.0, 0.5])
    assert float(result[2]) == pytest.approx(0.97)


def test_table_guard_checks_between_joint_endpoints():
    model = make_default_arm("right")
    model.joints[0].origin_xyz = np.asarray([0.2, 0.0, 0.8])
    model.joints[1].origin_xyz = np.asarray([1.1, 0.0, 0.0])
    for joint in model.joints[2:]:
        joint.origin_xyz = np.zeros(3)
    model.tcp_offset = np.zeros(3)

    guard = TableGuard(h_table=1.0, x0=0.35, x1=1.2,
                       y_half=0.5, margin=0.02)
    assert guard.collides_with_table(model, np.zeros(7))


def test_table_guard_rejects_invalid_geometry():
    with pytest.raises(ValueError):
        TableGuard(x0=1.0, x1=0.5)
    with pytest.raises(ValueError):
        TableGuard(y_half=0.0)
    with pytest.raises(ValueError):
        TableGuard(margin=-0.01)
