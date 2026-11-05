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
