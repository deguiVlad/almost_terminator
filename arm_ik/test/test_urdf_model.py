# -*- coding: utf-8 -*-
"""Regression tests for URDF-driven TienKung arm model construction."""
import os
import sys
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_ik._math_compat import np, Rotation
from arm_ik.tree_from_tf import build_models_from_urdf
from arm_ik.kinematics import fk


def _urdf():
    links = ['waist_yaw_link'] + [f'link_{i}' for i in range(1, 8)] + ['tcp_link', 'L_base_link']
    xml = ['<robot name="test">']
    xml += [f'<link name="{name}"/>' for name in links]
    axes = ['0 1 0', '1 0 0', '0 0 1', '0 1 0', '0 0 1', '0 1 0', '1 0 0']
    names = ['shoulder_pitch_l_joint', 'shoulder_roll_l_joint',
             'shoulder_yaw_l_joint', 'elbow_pitch_l_joint',
             'elbow_yaw_l_joint', 'wrist_pitch_l_joint', 'wrist_roll_l_joint']
    for i, (name, axis) in enumerate(zip(names, axes), 1):
        parent = 'waist_yaw_link' if i == 1 else 'link_' + str(i - 1)
        xml.append(
            f'<joint name="{name}" type="revolute">'
            f'<parent link="{parent}"/>'
            f'<child link="link_{i}"/><origin xyz="0 0 -0.1" rpy="0 0 0"/>'
            f'<axis xyz="{axis}"/><limit lower="-1.2" upper="1.1" effort="10" velocity="2"/></joint>')
    xml.append('<joint name="tcp_fixed" type="fixed"><parent link="link_7"/><child link="tcp_link"/><origin xyz="0 0 -0.08" rpy="0 0 0"/></joint>')
    xml.append('<joint name="hand_fixed" type="fixed"><parent link="tcp_link"/><child link="L_base_link"/><origin xyz="0 0 0" rpy="3.14 0 -1.570796"/></joint>')
    xml.append('</robot>')
    return ''.join(xml)


def test_urdf_model_uses_joint_axes_limits_and_fixed_tcp_orientation():
    models = build_models_from_urdf(_urdf(), 'waist_yaw_link')
    assert set(models) == {'left'}
    m = models['left']
    assert len(m.joints) == 7
    assert m.names[0] == 'shoulder_pitch_l_joint'
    assert m.names[-1] == 'wrist_roll_l_joint'
    assert m.ids == [11, 12, 13, 14, 15, 16, 17]
    assert np.allclose(m.joints[0].axis, [0, 1, 0])
    assert np.allclose(m.joints[1].axis, [1, 0, 0])
    assert m.lower[0] == pytest.approx(-1.2)
    assert m.upper[-1] == pytest.approx(1.1)
    assert m.tcp_offset[2] == pytest.approx(-0.08)
    assert abs(m.tcp_rpy[2]) > 1.0
    T, _ = fk(m, np.zeros(7))
    assert np.allclose(T[:3, :3], Rotation.from_euler('xyz', m.tcp_rpy).as_matrix(), atol=1e-6)



def test_model_prefers_robot_tcp_link_names():
    from arm_ik.urdf_model import build_arm_model
    xml = _urdf().replace("L_base_link", "left_tcp_link")
    model = build_arm_model(xml, "left", base_link="waist_yaw_link")
    assert model.tip_link == "left_tcp_link"
    assert len(model.joints) == 7



def test_full_two_arm_urdf_builds_both_tienkung_chains():
    from arm_ik.urdf_model import build_models_from_urdf

    tags = ("l", "r")
    xml = ['<robot name="two_arms"><link name="waist_yaw_link"/>']
    names = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
             "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll")
    axes = ("0 1 0", "1 0 0", "0 0 1", "0 1 0",
            "0 0 1", "0 1 0", "1 0 0")
    for tag in tags:
        parent = "waist_yaw_link"
        for index, (name, axis) in enumerate(zip(names, axes), 1):
            child = f"{tag}_link_{index}"
            xml.append(f'<link name="{child}"/>')
            xml.append(
                f'<joint name="{name}_{tag}_joint" type="revolute">'
                f'<parent link="{parent}"/><child link="{child}"/>'
                f'<origin xyz="0 0 0.1" rpy="0 0 0"/>'
                f'<axis xyz="{axis}"/>'
                f'<limit lower="-1.0" upper="1.0" effort="10" velocity="2"/>'
                f'</joint>')
            parent = child
        tip = "left_tcp_link" if tag == "l" else "right_tcp_link"
        xml.append(f'<link name="{tip}"/>')
        xml.append(
            f'<joint name="{tag}_tcp_fixed" type="fixed">'
            f'<parent link="{parent}"/><child link="{tip}"/>'
            f'<origin xyz="0.05 0 0" rpy="0 0 0"/></joint>')
    xml.append("</robot>")

    models = build_models_from_urdf("".join(xml), "waist_yaw_link")
    assert set(models) == {"left", "right"}
    assert models["left"].names == [f"{name}_l_joint" for name in names]
    assert models["right"].names == [f"{name}_r_joint" for name in names]
    assert models["left"].ids == list(range(11, 18))
    assert models["right"].ids == list(range(21, 28))
    assert models["left"].tip_link == "left_tcp_link"
    assert models["right"].tip_link == "right_tcp_link"
