# -*- coding: utf-8 -*-
"""Офлайн-тесты построения дерева звеньев IK из сообщений /tf + /tf_static.

Модуль arm_ik.tree_from_tf не импортирует rclpy/tf2_ros: он принимает
TFMessage-подобные объекты. Здесь используются лёгкие namedtuple-заменители
с той же структурой полей (header.frame_id, child_frame_id,
transform.translation/rotation), что и настоящие сообщения ROS 2.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_ik._math_compat import np, Rotation  # noqa: E402


class _H:
    def __init__(self, frame_id):
        self.frame_id = frame_id


class _Tr:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Q:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Tf:
    def __init__(self, parent, child, xyz=(0, 0, 0), rotvec=(0, 0, 0)):
        q = Rotation.from_rotvec(np.asarray(rotvec, dtype=float)).as_quat()
        self.header = _H(parent)
        self.child_frame_id = child
        self.transform = type("X", (), {"translation": _Tr(*xyz),
                                        "rotation": _Q(q[0], q[1], q[2], q[3])})()


class _TFMsg:
    def __init__(self, transforms):
        self.transforms = list(transforms)


def _synth_tf(base="pelvis"):
    """Синтетическое дерево TF обеих рук (7 DOF), как его отдаёт симулятор:
    fixed-рёбра — в static, активные суставные — в dynamic."""
    static, dyn = [], []
    joint_names = []
    for tag in ("l", "r"):
        yy = 0.20 if tag == "l" else -0.20
        prev = base
        # фиксированный кронштейн от pelvis к плечу
        static.append(_TFMsg([_Tf(prev, f"shoulder_{tag}_mount", (0.0, yy, 1.28))]))
        prev = f"shoulder_{tag}_mount"
        offs = [(0.0, 0.0, 0.0), (0.18, 0.0, 0.0), (0.0, 0.0, 0.0),
                (0.20, 0.0, 0.0), (0.0, 0.0, 0.0), (0.12, 0.0, 0.0)]
        kws = ["pitch", "roll", "yaw", "pitch", "yaw", "roll"]
        groups = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                  "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll"]
        seq = [("shoulder_pitch", offs[0]), ("shoulder_roll", offs[1]),
               ("shoulder_yaw", offs[2]), ("elbow_pitch", offs[3]),
               ("elbow_yaw", offs[4]), ("wrist_pitch", offs[5]),
               ("wrist_roll", (0.10, 0.0, 0.0))]
        for kw, off in seq:
            jn = f"{kw}_{tag}_joint"
            lk = f"{kw}_{tag}_link"
            joint_names.append(jn)
            dyn.append(_TFMsg([_Tf(prev, lk, off)]))
            prev = lk
        # fixed-цепочка за запястьем -> кончик кисти (TCP)
        static.append(_TFMsg([_Tf(prev, f"hand_{tag}_link", (0.05, 0.0, 0.0))]))
    return static, dyn, joint_names


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
    static, dyn, _ = _synth_tf()
    edges = tf_edges(static + dyn)
    assert ("pelvis", "shoulder_l_mount") in edges or \
        any(p == "pelvis" and c.startswith("shoulder") for p, c in edges)
    consts = tf_constants(dyn, static)
    assert len(consts) >= len(edges) - 2   # дубли перекрываются, static приоритетнее


def test_build_models_from_tf_both_hands():
    from arm_ik.tree_from_tf import build_models_from_tf
    static, dyn, joint_names = _synth_tf()
    models = build_models_from_tf(static, dyn, joint_names, "pelvis")
    assert set(models) == {"left", "right"}
    m = models["right"]
    assert m.names == ["shoulder_pitch_r_joint", "shoulder_roll_r_joint",
                       "shoulder_yaw_r_joint", "elbow_pitch_r_joint",
                       "elbow_yaw_r_joint", "wrist_pitch_r_joint",
                       "wrist_roll_r_joint"]
    assert m.ids == [21, 22, 23, 24, 25, 26, 27]
    assert m.tip_link == "hand_r_link"
    # tcp_offset пришёл из fixed-ребра wrist_roll_r_link -> hand_r_link
    assert max(abs(float(v)) for v in m.tcp_offset) == pytest.approx(0.05, abs=1e-9)
    # лимиты — встроенные doc-значения (robot_description не передавался)
    lo = [float(v) for v in m.lower]
    assert lo[0] == pytest.approx(-2.97)      # shoulder_pitch
    assert lo[3] == pytest.approx(-2.62)      # elbow_pitch


def test_build_models_from_tf_geometry_matches_fk():
    """origin'ы, свёрнутые из TF-дерева, воспроизводят ту же FK-позу,
    что и умножение трансформов TF (при q=0)."""
    from arm_ik.tree_from_tf import build_models_from_tf
    from arm_ik.kinematics import fk
    static, dyn, joint_names = _synth_tf()
    models = build_models_from_tf(static, dyn, joint_names, "pelvis")
    m = models["left"]
    T, _ = fk(m, np.array([0.0] * 7))
    p = [float(v) for v in T[:3, 3]]
    # x: 0.18+0.20+0.12+0.10+0.05 (сумма смещений вдоль цепочки), y: 0.20, z: 1.28
    assert p[0] == pytest.approx(0.65, abs=1e-6)
    assert p[1] == pytest.approx(0.20, abs=1e-6)
    assert p[2] == pytest.approx(1.28, abs=1e-6)


def test_limits_from_topic_robot_description_string():
    """Лимиты парсятся из СТРОКИ топика /robot_description (не из файла)."""
    from arm_ik.tree_from_tf import limits_from_urdf
    xml = ('<robot name="t"><joint name="elbow_pitch_l_joint" type="revolute">'
           '<limit lower="-1.5" upper="0.5"/></joint>'
           '<joint name="fixed_one" type="fixed"><limit lower="-9" upper="9"/></joint></robot>')
    lim = limits_from_urdf(xml)
    assert lim == {"elbow_pitch_l_joint": (-1.5, 0.5)}


def test_empty_tf_returns_no_models():
    from arm_ik.tree_from_tf import build_models_from_tf
    assert build_models_from_tf([], [], [], "pelvis") == {}


def test_partial_tree_only_one_hand():
    from arm_ik.tree_from_tf import build_models_from_tf
    static, dyn, joint_names = _synth_tf()
    left_only_joints = [j for j in joint_names if j.endswith("_l_joint")]
    left_edges = [m for m in static + dyn
                  if any("_l_" in t.child_frame_id or "_l_" in t.header.frame_id
                         for t in m.transforms)]
    models = build_models_from_tf(left_edges, [], left_only_joints, "pelvis")
    assert set(models) == {"left"}
