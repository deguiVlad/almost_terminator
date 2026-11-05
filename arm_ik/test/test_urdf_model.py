# -*- coding: utf-8 -*-
"""Regression tests for exact URDF chain parsing."""

import pytest

from arm_ik._math_compat import np, Rotation
from arm_ik.urdf_model import build_models_from_urdf


def _test_urdf():
    parts = ['<robot name="tiangkung_test">', '<link name="waist_yaw_link"/>']
    joint_order = (
        ("shoulder_pitch", "0 1 0"),
        ("shoulder_roll", "1 0 0"),
        ("shoulder_yaw", "0 0 1"),
        ("elbow_pitch", "0 1 0"),
        ("elbow_yaw", "0 0 1"),
        ("wrist_pitch", "0 1 0"),
        ("wrist_roll", "1 0 0"),
    )
    for side, tag in (("left", "l"), ("right", "r")):
        parent = "waist_yaw_link"
        for index, (joint_name, axis) in enumerate(joint_order):
            child = f"{joint_name}_{tag}_link"
            parts.append(f'<link name="{child}"/>')
            parts.append(
                f'<joint name="{joint_name}_{tag}_joint" type="revolute">'
                f'<parent link="{parent}"/><child link="{child}"/>'
                f'<origin xyz="0 0 {-0.02 * (index + 1):.4f}" rpy="0 0 0"/>'
                f'<axis xyz="{axis}"/>'
                f'<limit lower="-{1.0 + index * 0.1:.2f}" '
                f'upper="{1.0 + index * 0.1:.2f}" effort="10" velocity="1"/>'
                f'</joint>')
            parent = child
        tcp = f"{side}_tcp_link"
        parts.append(f'<link name="{tcp}"/>')
        parts.append(
            f'<joint name="{side}_tcp_joint" type="fixed">'
            f'<parent link="{parent}"/><child link="{tcp}"/>'
            f'<origin xyz="0 0 -0.0848" rpy="0 0 0"/></joint>')
    parts.append("</robot>")
    return "".join(parts)


def test_builds_both_tcp_chains_and_motor_ids():
    models = build_models_from_urdf(_test_urdf())
    assert set(models) == {"left", "right"}
    assert models["left"].tip_link == "left_tcp_link"
    assert models["right"].tip_link == "right_tcp_link"
    assert models["left"].ids == list(range(11, 18))
    assert models["right"].ids == list(range(21, 28))
    assert models["left"].names[0] == "shoulder_pitch_l_joint"
    assert models["right"].names[-1] == "wrist_roll_r_joint"
    assert models["left"].joints[0].axis.tolist() == [0.0, 1.0, 0.0]
    assert models["left"].lower[3] == pytest.approx(-1.3)
    assert models["right"].upper[6] == pytest.approx(1.6)
    assert models["left"].tcp_offset[2] == pytest.approx(-0.0848)


def test_rejects_missing_tcp_chain():
    xml = '<robot name="empty"><link name="waist_yaw_link"/></robot>'
    with pytest.raises(ValueError, match="no path"):
        build_models_from_urdf(xml)


def test_rejects_zero_joint_axis():
    xml = _test_urdf().replace('axis xyz="0 1 0"', 'axis xyz="0 0 0"', 1)
    with pytest.raises(ValueError, match="invalid axis"):
        build_models_from_urdf(xml)


def test_xyz_euler_round_trip_in_math_compat():
    expected = np.asarray([0.2, -0.3, 0.4], dtype=float)
    actual = Rotation.from_matrix(
        Rotation.from_euler("xyz", expected).as_matrix()).as_euler("xyz")
    assert np.allclose(actual, expected, atol=1e-8)
