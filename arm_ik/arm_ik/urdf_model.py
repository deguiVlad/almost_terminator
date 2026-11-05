# -*- coding: utf-8 -*-
"""Build exact 7-DOF arm models from a URDF XML document.

The URDF is the source of joint origins, axes, limits and the complete fixed
transform from the final wrist joint to the selected hand TCP frame.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from arm_ik._math_compat import np, Rotation
from arm_ik.kinematics import ArmModel, JointSpec, _tmat
from arm_ik.tree_from_tf import motor_id_for

_ARM_JOINTS = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll",
)
_TIP_LINK_CANDIDATES = {
    "left": ("left_tcp_link", "L_base_link"),
    "right": ("right_tcp_link", "R_base_link"),
}


def _vec(text, default):
    if not text:
        return np.asarray(default, dtype=float)
    values = [float(v) for v in text.split()]
    if len(values) != 3:
        raise ValueError("URDF vector must contain exactly 3 numbers")
    return np.asarray(values, dtype=float)


def _origin(joint):
    origin = joint.find("origin")
    if origin is None:
        return np.eye(4)
    return _tmat(_vec(origin.get("xyz"), [0, 0, 0]),
                 _vec(origin.get("rpy"), [0, 0, 0]))


def _path_to_tip(joint_by_child, base_link, tip_link):
    path = []
    link = tip_link
    visited = set()
    while link != base_link:
        if link in visited:
            raise ValueError("cycle in URDF while resolving " + tip_link)
        visited.add(link)
        joint = joint_by_child.get(link)
        if joint is None:
            raise ValueError(
                "URDF has no path from %s to %s (stopped at %s)" %
                (base_link, tip_link, link))
        path.append(joint)
        link = joint.find("parent").get("link")
    return list(reversed(path))


def build_arm_model(urdf_xml, side, base_link="waist_yaw_link",
                    tip_link=None):
    """Parse one arm model; fixed joints are folded into adjacent origins/TCP."""
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    root = ET.fromstring(urdf_xml)
    if tip_link is None:
        available_links = {link.get("name") for link in root.findall("link")}
        tip_link = next(
            (candidate for candidate in _TIP_LINK_CANDIDATES[side]
             if candidate in available_links),
            None)
        if tip_link is None:
            raise ValueError(
                "URDF has no known %s TCP link; expected one of %s" %
                (side, _TIP_LINK_CANDIDATES[side]))
    joints_xml = list(root.findall("joint"))
    joint_by_child = {}
    for joint in joints_xml:
        child = joint.find("child")
        if child is None:
            continue
        child_name = child.get("link")
        if child_name in joint_by_child:
            raise ValueError("multiple parent joints for link " + child_name)
        joint_by_child[child_name] = joint

    path = _path_to_tip(joint_by_child, base_link, tip_link)
    tag = "l" if side == "left" else "r"
    expected = [name + "_" + tag + "_joint" for name in _ARM_JOINTS]
    movable = [j for j in path if j.get("type") in ("revolute", "continuous")]
    names = [j.get("name") for j in movable]
    if names != expected:
        raise ValueError(
            "%s chain must contain exactly %s in order; found %s" %
            (side, expected, names))

    pending = np.eye(4)
    specs = []
    for joint in path:
        T_origin = _origin(joint)
        if joint.get("type") == "fixed":
            pending = pending @ T_origin
            continue
        if joint.get("type") not in ("revolute", "continuous"):
            raise ValueError("unsupported joint type %s in arm chain" % joint.get("type"))
        T_joint = pending @ T_origin
        xyz = T_joint[:3, 3].copy()
        rpy = Rotation.from_matrix(T_joint[:3, :3]).as_euler("xyz")
        axis_el = joint.find("axis")
        axis = _vec(axis_el.get("xyz") if axis_el is not None else None,
                    [1.0, 0.0, 0.0])
        norm = float(np.linalg.norm(axis))
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("invalid axis for " + joint.get("name", "unnamed joint"))
        axis = axis / norm
        limit = joint.find("limit")
        if joint.get("type") == "continuous":
            lower, upper = -math.pi, math.pi
        elif limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        else:
            raise ValueError("missing lower/upper limits for " + joint.get("name", "unnamed joint"))
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError("invalid limits for " + joint.get("name", "unnamed joint"))
        motor_id = motor_id_for(joint.get("name", ""))
        if motor_id <= 0:
            raise ValueError("no motor ID mapping for " + joint.get("name", "unnamed joint"))
        specs.append(JointSpec(
            name=joint.get("name"), motor_id=motor_id, axis=axis,
            origin_xyz=xyz, origin_rpy=np.asarray(rpy, dtype=float),
            lower=lower, upper=upper))
        pending = np.eye(4)

    tcp_xyz = pending[:3, 3].copy()
    tcp_rpy = Rotation.from_matrix(pending[:3, :3]).as_euler("xyz")
    return ArmModel(
        side=side, joints=specs, tip_link=tip_link, base_frame=base_link,
        tcp_offset=tcp_xyz, tcp_rpy=np.asarray(tcp_rpy, dtype=float))


def build_models_from_urdf(urdf_xml, base_link="waist_yaw_link"):
    """Return {'left': ArmModel, 'right': ArmModel} from the same URDF."""
    return {
        side: build_arm_model(urdf_xml, side, base_link=base_link)
        for side in ("left", "right")
    }
