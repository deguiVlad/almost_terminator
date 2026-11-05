"""Legacy TF-message inspection helpers.

TF transforms are useful for frame conversion and diagnostics, but they do not
contain the original URDF joint axes. Consequently this module never builds an
IK model from TF-only data. Runtime arm models are built by arm_ik.urdf_model.
"""Кинематическое дерево звеньев рук из топиков симулятора /tf и /tf_static.

Вместо файлов (config/arms.json, URDF на диске) структура звеньев берётся из
живого TF2-дерева:

  * рёбра /tf_static — постоянные смещения звеньев (fixed-соединения URDF);
  * рёбра /tf        — активные (вращательные) соединения; их child-рамки
    сопоставляются с именами суставов из /joint_states по суффиксу стороны
    (_l/_left, _r/_right);
  * пределы суставов — из топика /robot_description (лимиты, переданные
    симулятором), затем — встроенные типовые значения (docs/INTERFACES.md),
    иначе ±pi. Файлы с диска не читаются.

Модуль НЕ импортирует rclpy/tf2_ros на уровне модуля: данные принимаются
обычными кортежами (см. tf_edges / tf_constants), поэтому тестируется офлайн.
ROS-нода извлекает кортежи из tf2_ros.Buffer.all_as_listofmessages().

Выход — dict внутренней конфигурации руки, который читает
arm_ik.kinematics.arm_from_dict (файл-конфиг для этого не нужен).
"""
from __future__ import annotations

import collections
import math

from arm_ik._math_compat import np
from arm_ik._math_compat import Rotation

# Ключевые слова цепочки плечо -> запястье (7 DOF на руку), сверху вниз.
KEYWORDS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll"]

# Штатные motor_id рук TienKung 2.0 Pro (docs/INTERFACES.md).
MOTOR_IDS = {}
for _i, _kw in enumerate(KEYWORDS):
    MOTOR_IDS[f"{_kw}_l_joint"] = 11 + _i
    MOTOR_IDS[f"{_kw}_r_joint"] = 21 + _i


def _norm(frame):
    """Срезает ведущий '/' и суффиксы _link/_joint для сопоставления имён."""
    f = frame[1:] if frame.startswith("/") else frame
    return f


def _stem(name):
    """'wrist_roll_l_link' / 'wrist_roll_l_joint' -> 'wrist_roll_l'."""
    n = _norm(name)
    for suf in ("_link", "_joint"):
        if n.endswith(suf):
            return n[:-len(suf)]
    return n


def side_of(name):
    """'left' | 'right' | None по суффиксу/ infix имени рамки или сустава."""
    s = _stem(name).lower()
    has_l = "_l_" in s or s.endswith("_l") or "_left" in s or s.endswith("_left")
    has_r = ("_r_" in s or s.endswith("_r") or "_right" in s
             or s.endswith("_right"))
    # защита от ложных срабатываний (например 'shoulder' содержит 'r')
    toks = set(s.split("_"))
    has_l = has_l and ("l" in toks or "left" in toks)
    has_r = has_r and ("r" in toks or "right" in toks)
    if has_l and not has_r:
        return "left"
    if has_r and not has_l:
        return "right"
    return None




# ---------------------------------------------------------------------------
# Извлечение сырых данных из сообщений TF (работает и с msg-объектами ROS,
# и с простыми namedtuple-заменителями в тестах)
# ---------------------------------------------------------------------------

def _child_of(t):
    """child-рамка TransformStamped (в ROS 2 поле на верхнем уровне,
    в старых версиях — внутри header)."""
    c = getattr(t, "child_frame_id", None)
    if c is None:
        hdr = getattr(t, "header", None)
        c = getattr(hdr, "child_frame_id", "") if hdr is not None else ""
    return c or ""


def _iter_transforms(msgs):
    """Разворачивает список TFMessage/TransformStamped в поток трансформов."""
    for m in msgs or []:
        trs = getattr(m, "transforms", None)
        if trs is None and hasattr(m, "header"):
            trs = [m]
        for t in trs or []:
            yield t


def tf_edges(msgs):
    """[(parent_frame, child_frame)] по сообщениям TF (без дубликатов)."""
    out, seen = [], set()
    for t in _iter_transforms(msgs):
        p, c = _norm(t.header.frame_id), _norm(_child_of(t))
        if p and c and p != c and (p, c) not in seen:
            seen.add((p, c))
            out.append((p, c))
    return out


def tf_pose(t):
    """(T 4x4, xyz list, rpy list) из TransformStamped-подобного объекта."""
    tr, ro = t.transform.translation, t.transform.rotation
    xyz = [tr.x, tr.y, tr.z]
    q = [ro.x, ro.y, ro.z, ro.w]
    R = Rotation.from_quat(q).as_matrix()
    rpy = [float(v) for v in Rotation.from_matrix(R).as_euler("xyz")]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T, xyz, rpy


def tf_constants(static_msgs, dynamic_msgs):
    """{(parent, child): (T_parent_child при q=0)} — геометрия дерева.

    Для fixed-рёбер это точный постоянный transform; для активных рёбер —
    transform «как есть» (нужен только для определения ориентации рамки,
    см. axis_of_edge).
    """
    table = {}
    for src in (dynamic_msgs, static_msgs):   # static имеет приоритет
        for t in _iter_transforms(src):
            p, c = _norm(t.header.frame_id), _norm(_child_of(t))
            if p and c and p != c:
                T, _xyz, _rpy = tf_pose(t)
                table[(p, c)] = T
    return table


# ---------------------------------------------------------------------------
# Обход дерева
# ---------------------------------------------------------------------------

def motor_id_for(joint_name):
    jn = joint_name.lower()
    if jn in MOTOR_IDS:
        return MOTOR_IDS[jn]
    sd = side_of(joint_name)
    tag = "l" if sd == "left" else ("r" if sd == "right" else None)
    stem = _stem(joint_name)
    if tag:
        for kw in KEYWORDS:                       # shoulder_pitch_l и т.п.
            if stem.endswith(kw + "_" + tag):
                cand = f"{kw}_{tag}_joint"
                if cand in MOTOR_IDS:
                    return MOTOR_IDS[cand]
    for full, v in MOTOR_IDS.items():             # fallback по суффиксу
        if _stem(full) == stem:
            return v
    return 0


def build_models_from_urdf(urdf_xml, base_frame="waist_yaw_link", logger=None):
    """Compatibility wrapper around the canonical URDF parser."""
    from arm_ik.urdf_model import build_arm_model
    models = {}
    for side in ("left", "right"):
        try:
            models[side] = build_arm_model(
                urdf_xml, side, base_link=base_frame)
        except ValueError as exc:
            if "no path from" in str(exc) or "no known" in str(exc):
                continue
            raise
        if logger is not None:
            logger.info("[%s] 7-DOF model loaded from URDF; base=%s" %
                        (side, base_frame))
    return models


# ---------------------------------------------------------------------------
# Пределы суставов
# ---------------------------------------------------------------------------

def limits_from_urdf(urdf_xml):
    """{joint_name: (lower, upper)} из строки /robot_description (xml.etree,
    без внешних зависимостей; это топик симулятора, а НЕ файл на диске)."""
    import xml.etree.ElementTree as ET
    out = {}
    if not urdf_xml:
        return out
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for j in root.iter("joint"):
        if j.get("type") in (None, "fixed", "floating", "planar"):
            continue
        el = j.find("limit")
        if el is None:
            continue
        try:
            lo = float(el.get("lower", "-3.14"))
            up = float(el.get("upper", "3.14"))
        except ValueError:
            continue
        if j.get("name"):
            out[j.get("name")] = (lo, up)
    return out


def build_models_from_tf(static_msgs, dynamic_msgs, joint_names, base_frame,
                         urdf_xml=None, tip_hints=None, tcp_offsets=None,
                         logger=None):
    """Deprecated compatibility entry point.

    Joint axes cannot be recovered from current TF transforms. Use a URDF
    description; TF-only reconstruction is rejected rather than returning a
    plausible-looking but physically incorrect arm model.
    """
    if not urdf_xml:
        raise ValueError(
            "TF-only arm model reconstruction is unsafe: provide the URDF XML "
            "through urdf_xml or use arm_ik.urdf_model.build_models_from_urdf")
    from arm_ik.urdf_model import build_models_from_urdf
    models = build_models_from_urdf(urdf_xml, base_link=base_frame)
    if logger is not None:
        logger.info("Built arm models from URDF; TF data is not used for joint axes")
    return models
