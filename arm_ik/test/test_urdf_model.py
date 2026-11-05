# -*- coding: utf-8 -*-
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_ik._math_compat import np, Rotation
from arm_ik.kinematics import fk, geometric_jacobian
from arm_ik.urdf_model import build_models_from_urdf


def _sample_urdf():
    links = ["waist_yaw_link"]
    joints = []
    parent = "waist_yaw_link"
    for side, tag, y, rpy in (("left", "l", 0.1444, "0.087266 0 0"),
                              ("right", "r", -0.1444, "-0.087266 0 0")):
        parent = "waist_yaw_link"
        for i, kw in enumerate(("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                                "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll")):
            child = kw + "_" + tag + "_link"
            name = kw + "_" + tag + "_joint"
            xyz = "0 %s 0.37712" % y if i == 0 else "0 0 -0.1"
            origin_rpy = rpy if i == 0 else "0 0 0"
            axis = ("0 1 0", "1 0 0", "0 0 1", "0 1 0",
                    "0 0 1", "0 1 0", "1 0 0")[i]
            joints.append('<joint name="%s" type="revolute"><origin xyz="%s" rpy="%s"/>'
                          '<parent link="%s"/><child link="%s"/><axis xyz="%s"/>'
                          '<limit lower="-1.5" upper="1.5"/></joint>' %
                          (name, xyz, origin_rpy, parent, child, axis))
            parent = child
        tip = "L_base_link" if side == "left" else "R_base_link"
        hand_rpy = "3.14 0 -1.570796" if side == "left" else "3.14 0 1.570796"
        joints.append('<joint name="%s_tcp_fixed" type="fixed"><origin xyz="0 0 -0.0848" '
                      'rpy="0 0 0"/><parent link="%s"/><child link="%s"/></joint>' %
                      (side, parent, tip))
        # Add a second fixed hand transform to exercise full 6D TCP orientation.
        palm = "L_palm" if side == "left" else "R_palm"
        joints.append('<joint name="%s_hand_fixed" type="fixed"><origin xyz="0 0 0" '
                      'rpy="%s"/><parent link="%s"/><child link="%s"/></joint>' %
                      (side, hand_rpy, tip, palm))
    return '<robot name="test">' + "".join(joints) + '</robot>'


def test_urdf_builds_both_seven_dof_models_and_motor_ids():
    models = build_models_from_urdf(_sample_urdf())
    assert models["left"].names == [
        "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint",
        "wrist_roll_l_joint"]
    assert models["left"].ids == list(range(11, 18))
    assert models["right"].ids == list(range(21, 28))
    assert models["left"].tip_link == "L_base_link"
    assert models["right"].tip_link == "R_base_link"
    assert models["left"].tcp_offset[2] == pytest.approx(-0.0848)
    assert abs(float(models["left"].tcp_rpy[2])) == pytest.approx(1.570796, abs=1e-5)


def test_urdf_axes_limits_and_fk_are_used():
    models = build_models_from_urdf(_sample_urdf())
    left = models["left"]
    assert np.allclose(left.joints[0].axis, [0, 1, 0])
    assert left.lower.tolist() == pytest.approx([-1.5] * 7)
    T, _ = fk(left, np.zeros(7))
    assert np.isfinite(T).all()
    assert geometric_jacobian(left, np.zeros(7)).shape == (6, 7)


def test_urdf_rejects_missing_arm_joint():
    xml = _sample_urdf().replace('name="wrist_roll_r_joint"', 'name="broken_wrist_roll_r_joint"')
    with pytest.raises(ValueError):
        build_models_from_urdf(xml)
