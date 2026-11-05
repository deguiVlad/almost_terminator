# -*- coding: utf-8 -*-
"""Regression tests for TF parsing and the fail-closed TF-only model API."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class _Header:
    def __init__(self, frame_id):
        self.frame_id = frame_id


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    x, y, z, w = 0.0, 0.0, 0.0, 1.0


class _Transform:
    def __init__(self, parent, child):
        self.header = _Header(parent)
        self.child_frame_id = child
        self.transform = type("TransformData", (), {
            "translation": _Vector(),
            "rotation": _Quaternion(),
        })()


class _TFMessage:
    def __init__(self, transforms):
        self.transforms = transforms


def test_side_of_and_stem():
    from arm_ik.tree_from_tf import side_of, _stem
    assert side_of("wrist_roll_l_link") == "left"
    assert side_of("shoulder_pitch_r_joint") == "right"
    assert side_of("pelvis") is None
    assert side_of("camera_head_link") is None
    assert _stem("wrist_roll_l_link") == "wrist_roll_l"
    assert _stem("wrist_roll_l_joint") == "wrist_roll_l"


def test_tf_edges_and_constants():
    from arm_ik.tree_from_tf import tf_edges, tf_constants
    static = [_TFMessage([_Transform("waist_yaw_link", "shoulder_l_mount")])]
    dynamic = [_TFMessage([_Transform("shoulder_l_mount", "shoulder_pitch_l_link")])]
    edges = tf_edges(static + dynamic)
    assert edges == [
        ("waist_yaw_link", "shoulder_l_mount"),
        ("shoulder_l_mount", "shoulder_pitch_l_link"),
    ]
    constants = tf_constants(dynamic, static)
    assert set(constants) == set(edges)


def test_tf_only_model_reconstruction_is_rejected():
    from arm_ik.tree_from_tf import build_models_from_tf
    with pytest.raises(ValueError, match="TF-only arm model reconstruction is unsafe"):
        build_models_from_tf([], [], [], "waist_yaw_link")


def test_limits_from_robot_description_string():
    from arm_ik.tree_from_tf import limits_from_urdf
    xml = ('<robot name="t"><joint name="elbow_pitch_l_joint" type="revolute">'
           '<limit lower="-1.5" upper="0.5"/></joint>'
           '<joint name="fixed_one" type="fixed"><limit lower="-9" upper="9"/></joint></robot>')
    assert limits_from_urdf(xml) == {"elbow_pitch_l_joint": (-1.5, 0.5)}
