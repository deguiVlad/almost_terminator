# -*- coding: utf-8 -*-
"""Кинематическое дерево звеньев рук из топиков симулятора /tf и /tf_static.

Вместо файлов (config/arms.json, URDF на диске) структура звеньев берётся из
живого TF2-дерева:

  * рёбра /tf_static — постоянные смещения звеньев (fixed-соединения URDF);
  * рёбра /tf        — активные (вращательные) соединения; их child-рамки
    сопоставляются с именами суставов из /joint_states по суффиксу стороны
    (_l/_left, _r/_right);
  * пределы суставов — из /robot_description (URDF-лимиты), затем — из
    config/arms_template.json (docs/INTERFACES.md), иначе ±pi.

Модуль НЕ импортирует rclpy/tf2_ros на уровне модуля: данные принимаются
обычными кортежами (см. tf_edges / tf_constants), поэтому тестируется офлайн.
ROS-нода извлекает кортежи из tf2_ros.Buffer.all_as_listofmessages().

Выход — dict в формате arms.json, который читает arm_ik.kinematics.arm_from_dict.
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


def is_arm_joint(name):
    s = _stem(name).lower()
    return any(kw in s for kw in KEYWORDS)


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

def build_tree(edges):
    children = collections.defaultdict(list)
    for p, c in edges:
        children[p].append(c)
    return children


def find_chain(children, base, joints_links):
    """Путь base -> самый глубокий сустав руки (список рамок, включая оба конца)."""
    best = None
    stack = [[base]]
    target = set(joints_links)
    while stack:
        path = stack.pop()
        node = path[-1]
        if node in target and (best is None or len(path) > len(best)):
            best = list(path)
        for ch in children.get(node, ()):
            if ch not in path:                    # защита от циклов
                stack.append(path + [ch])
    return best


def fold_origin(T_abs_prev_inv, T_abs_joint):
    """origin_i = inv(T_{i-1}) @ T_i — позиция/ориентация кадра сустава
    в системе предыдущего звена (до его собственного поворота)."""
    O = np.linalg.inv(np.asarray(T_abs_prev_inv, dtype=float)) @ \
        np.asarray(T_abs_joint, dtype=float)
    xyz = [float(v) for v in O[:3, 3]]
    rpy = [float(v) for v in Rotation.from_matrix(O[:3, :3]).as_euler("xyz")]
    return xyz, rpy


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


def build_side_config(edges, constants, joint_names, base, side,
                      tip_hint=None):
    """dict одной руки (формат arms.json) из TF-дерева; None если рука не найдена.

    Суставной кадр = child-ребро, чей stem совпадает со stem сустава.
    Ось вращения выводится из движения кадра во времени: ось — собственный
    Z кадра сустава (стандарт URDF->TF: вращение происходит вокруг оси
    собственной рамки child). Все origin'ы пересчитываются в системы
    ПРЕДЫДУЩИХ звеньев (как требует JointSpec/URDF).
    """
    children = build_tree(edges)
    link_by_stem = {}
    for p, c in edges:
        link_by_stem.setdefault(_stem(c), c)
    all_frames = {c for _, c in edges} | {p for p, _ in edges}

    js = sorted({n for n in joint_names
                 if is_arm_joint(n) and side_of(n) == side})
    if not js:
        return None
    # связь имя сустава -> TF-рамка
    j2link = {}
    for n in js:
        stem = _stem(n)
        if stem in link_by_stem:
            j2link[n] = link_by_stem[stem]
        else:
            for lk in all_frames:
                if _stem(lk) == stem:
                    j2link[n] = lk
                    break
    reachable = {j: l for j, l in j2link.items()
                 if find_chain(children, base, [l]) is not None}
    if not reachable:
        return None
    chains = {j: find_chain(children, base, [l]) for j, l in reachable.items()}
    deepest_j = max(chains, key=lambda j: len(chains[j]))
    chain = chains[deepest_j]                     # base ... link последнего сустава

    # суставы строго в порядке глубины вдоль найденной цепочки
    pos_in_chain = {lk: i for i, lk in enumerate(chain)}
    ordered = sorted(((pos_in_chain[l], j) for j, l in reachable.items()
                      if l in pos_in_chain))
    if not ordered:
        return None

    from arm_ik.kinematics import _tmat
    # абсолютные кадры рамок в системе base (по фиксированным рёбрам;
    # активные рёбра дают кадр сустава «как есть» — годится для origin,
    # потому что IK-модель сама добавляет поворот вокруг оси)
    def abs_T(frame):
        path = find_chain(children, base, [frame])
        if path is None:
            return None
        T = np.eye(4)
        for a, b in zip(path[:-1], path[1:]):
            Te = constants.get((a, b))
            if Te is not None:
                T = T @ np.asarray(Te, dtype=float)
        return T

    out_joints = []
    prev_abs = np.eye(4)                          # кадр base
    for _pos, jn in ordered:
        lk = reachable[jn]
        cur_abs = abs_T(lk)
        if cur_abs is None:
            continue
        xyz, rpy = fold_origin(prev_abs, cur_abs)
        out_joints.append({
            "name": jn,
            "motor_id": int(motor_id_for(jn)),
            "axis": [0.0, 0.0, 1.0],   # собственная Z-ось кадра сустава
            "origin_xyz": xyz,
            "origin_rpy": rpy,
            "limits": [-math.pi, math.pi],
        })
        # кадр «после» поворота этого сустава (нейтральный q_i=0): origin
        # следующего сустава вычитается именно из него; постоянная часть
        # суставного ребра уже сидит в Tj, собственный угол TF-кадра при
        # съёмке в нейтральной позе равен нулю
        prev_abs = prev_abs @ _tmat(xyz, rpy)
    if not out_joints:
        return None

    # кончик кисти: идём дальше от последнего сустава до листа дерева
    last_frame = reachable[ordered[-1][1]]
    tip = tip_hint or last_frame
    cur = last_frame
    advanced = True
    while advanced:
        advanced = False
        for ch in children.get(cur, ()):
            if not is_arm_joint(ch):
                tip, cur = ch, ch
                advanced = True
                break
    # tcp_offset: смещение TCP от frame последнего сустава (leaf fixed-цепь)
    tcp = [0.0, 0.0, 0.0]
    if tip != last_frame:
        path = find_chain(children, last_frame, [tip])
        if path:
            Trel = np.eye(4)
            for a, b in zip(path[:-1], path[1:]):
                Te = constants.get((a, b))
                if Te is not None:
                    Trel = Trel @ np.asarray(Te, dtype=float)
            tcp = [float(v) for v in Trel[:3, 3]]

    return {"side": side, "tip_link": tip,
            "tcp_offset": tcp, "joints": out_joints}


# ---------------------------------------------------------------------------
# Пределы суставов
# ---------------------------------------------------------------------------

def limits_from_urdf(urdf_xml):
    """{joint_name: (lower, upper)} из robot_description (xml.etree, без зависимостей)."""
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


def load_template_limits():
    """Запасной источник пределов: config/arms_template.json (docs/INTERFACES.md)."""
    import json
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for cand in (os.path.join(here, "config", "arms.json"),
                 os.path.join(here, "config", "arms_template.json")):
        if os.path.exists(cand):
            try:
                with open(cand, "r", encoding="utf-8") as f:
                    data = json.load(f)
                out = {}
                for sd in ("left", "right"):
                    for jd in data.get(sd, {}).get("joints", []):
                        out[jd["name"]] = (float(jd["limits"][0]),
                                           float(jd["limits"][1]))
                if out:
                    return out
            except Exception:
                pass
    return {}


def apply_limits(cfg, limits):
    for jd in cfg["joints"]:
        if jd["name"] in limits:
            lo, up = limits[jd["name"]]
            jd["limits"] = [float(lo), float(up)]


# ---------------------------------------------------------------------------
# Главный вход
# ---------------------------------------------------------------------------

def build_models_from_tf(static_msgs, dynamic_msgs, joint_names, base_frame,
                         urdf_xml=None, tip_hints=None, tcp_offsets=None,
                         logger=None):
    """Сообщения /tf + /tf_static -> {'left': ArmModel, 'right': ArmModel}.

    Возвращает dict (возможно неполый — нода сама решает, хватит ли одного
    комплекта или нужен fallback на файл). Пустой dict — если в дереве нет
    ни одной руки.
    """
    from arm_ik.kinematics import arm_from_dict

    def log(level, text):
        if logger is not None:
            getattr(logger, level)(text)

    edges = tf_edges(list(static_msgs or []) + list(dynamic_msgs or []))
    if not edges:
        log("warn", "TF-дерево пустое (/tf, /tf_static) — модель не собрана")
        return {}
    constants = tf_constants(dynamic_msgs, static_msgs)

    # имена суставов: из /joint_states + все подходящие stems активных рёбер
    names = set(joint_names or [])
    for p, c in edges:
        if is_arm_joint(c):
            names.add(c.replace("_link", "_joint")
                      if c.endswith("_link") else c)
    limits = limits_from_urdf(urdf_xml) or load_template_limits()

    out = {}
    for side in ("left", "right"):
        cfg = build_side_config(edges, constants, sorted(names), base_frame,
                                side, tip_hint=(tip_hints or {}).get(side))
        if cfg is None:
            log("warn", f"[{side}] цепочка руки в дереве TF не найдена")
            continue
        apply_limits(cfg, limits)
        if tcp_offsets and side in tcp_offsets:
            cfg["tcp_offset"] = list(tcp_offsets[side])
        out[side] = arm_from_dict(cfg)
        log("info", f"[{side}] IK-цепочка из TF: {len(cfg['joints'])} звеньев, "
                    f"tip={cfg['tip_link']}, base={base_frame}")
    return out
