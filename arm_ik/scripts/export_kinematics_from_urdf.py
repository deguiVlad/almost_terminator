#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Экспорт кинематики рук из URDF робота в config/arms.json.

Запуск внутри контейнера симулятора (или где есть доступ к файлам проекта):

    # 1) из файла URDF (в nssr-polygon файл называется так):
    python3 export_kinematics_from_urdf.py \
        --urdf ~/nssr-polygon/assets/robots/tiangong_pro/tiangong_pro_with_hands.urdf \
        --out ../config/arms.json

    # 2) с живого робота — взять /robot_description (URDF опубликован там же):
    ros2 topic echo --once /robot_description data | ... (см. README)

Скрипт парсит XML без дополнительных зависимостей (xml.etree), находит цепочки
плечо→запястье для обеих рук, выписывает origin/axis/limits каждого сустава и
пишет JSON, который читает arm_ik.kinematics.load_arm_model().
"""
import argparse
import json
import xml.etree.ElementTree as ET

MOTOR_IDS = {  # по docs/INTERFACES.md
    "shoulder_pitch_l_joint": 11, "shoulder_roll_l_joint": 12, "shoulder_yaw_l_joint": 13,
    "elbow_pitch_l_joint": 14, "elbow_yaw_l_joint": 15, "wrist_pitch_l_joint": 16,
    "wrist_roll_l_joint": 17,
    "shoulder_pitch_r_joint": 21, "shoulder_roll_r_joint": 22, "shoulder_yaw_r_joint": 23,
    "elbow_pitch_r_joint": 24, "elbow_yaw_r_joint": 25, "wrist_pitch_r_joint": 26,
    "wrist_roll_r_joint": 27,
}


def find_urdf(explicit):
    """Если --urdf не задан, ищем файл типовыми путями."""
    import os, glob
    if explicit:
        return explicit
    candidates = []
    for root in [os.path.expanduser("~/nssr-polygon"), os.path.expanduser("~"), "/nssr_sim", "/workspace"]:
        candidates += glob.glob(os.path.join(root, "**/tiangong_pro*_with_hands.urdf"), recursive=True)
        candidates += glob.glob(os.path.join(root, "**/tiangong_pro*/robot*.urdf"), recursive=True)
    for c in candidates:
        if os.path.isfile(c):
            print(f"URDF найден автоматически: {c}")
            return c
    raise SystemExit("Не найден URDF. Укажите --urdf явно, напр.:\n"
                     "  --urdf ~/nssr-polygon/assets/robots/tiangong_pro/tiangong_pro_with_hands.urdf")


def chain_joint_names(root, known_revolute):
    """Известные revolute-суставы рук + все fixed внутри кисти (для tip_link)."""
    names = set(known_revolute)
    return names


def parse_origin(el):
    if el is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in (el.get("xyz", "0 0 0").split())]
    rpy = [float(v) for v in (el.get("rpy", "0 0 0").split())]
    return xyz, rpy


def build_config(urdf_path, base="pelvis", tcp=(0.05, 0.0, 0.0),
                 tip_link_left=None, tip_link_right=None):
    """Парсит URDF и возвращает dict конфига arms.json (left/right цепочки по 7 суставов)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = {}
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            continue  # фиксированные связи обрабатываются отдельно при поиске tip_link
        axis_el = j.find("axis")
        axis = [float(v) for v in axis_el.get("xyz", "0 0 1").split()] if axis_el is not None else [0, 0, 1]
        lim_el = j.find("limit")
        lo = float(lim_el.get("lower", "-3.14")) if lim_el is not None else -3.14
        up = float(lim_el.get("upper", "3.14")) if lim_el is not None else 3.14
        xyz, rpy = parse_origin(j.find("origin"))
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            axis=axis, origin_xyz=xyz, origin_rpy=rpy, limits=[lo, up])

    import re as _re
    KEYWORDS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll"]

    SIDE_WORDS = {"l": ["_l_", "_left_", "l_joint", "left_joint"],
                  "r": ["_r_", "_right_", "r_joint", "right_joint"]}

    def side_matches(name, tag):
        n = name.lower()
        return any(w in n for w in SIDE_WORDS[tag])

    def resolve_name(kw, tag):
        # 1) точное имя из docs/INTERFACES.md
        cand = [f"{kw}_{tag}_joint", f"{kw}_{tag}", f"{kw}_{tag}_axis"]
        for c in cand:
            if c in joints:
                return c
        # 2) все суставы с этим ключевым словом, затем фильтр по метке стороны
        kws = [n for n in joints if kw in n.lower()]
        tagged = [n for n in kws if side_matches(n, tag)]
        if len(tagged) == 1:
            return tagged[0]
        if not kws:
            raise SystemExit(
                f"В URDF вообще нет сустава со словом '{kw}'. "
                f"Список суставов: {sorted(joints)}\n"
                f"Подправьте KEYWORDS/MOTOR_IDS в скрипте под фактические имена.")
        # 3) если в файле представлена только одна рука (например, _l во всех
        #    именах), для второй стороны берём тот же шаблон — геометрика
        #    зеркальная, motor_id подставляются из MOTOR_IDS по тегу.
        if len(kws) == 1 and not tagged:
            return kws[0]
        raise SystemExit(
            f"Неоднозначно: '{kw}' для стороны '{tag}' -> кандидаты {kws}, "
            f"после фильтра стороны {tagged}")

    def chain_for(tag):
        out = []
        for kw in KEYWORDS:
            n = resolve_name(kw, tag)
            jd = dict(joints[n])
            jd["name"] = n
            mid = MOTOR_IDS.get(f"{kw}_{tag}_joint", 0)
            if mid == 0:  # fallback: по суффиксу фактического имени сустава
                for full, v in MOTOR_IDS.items():
                    if full.endswith(n):
                        mid = v
                        break
            jd["motor_id"] = mid
            out.append(jd)
        return out

    def find_tip_link(chain):
        """Идём от child последнего сустава вниз по fixed-суставам до листа — это кончик кисти."""
        cur = chain[-1]["child"]
        fixed_by_parent = {}
        for j in root.findall("joint"):
            if j.get("type") == "fixed":
                fixed_by_parent.setdefault(j.find("parent").get("link"), []).append(j)
        while True:
            nxt = fixed_by_parent.get(cur, [])
            if not nxt:
                return cur
            cur = nxt[0].find("child").get("link")

    tcp = [float(v) for v in (tcp if not isinstance(tcp, str) else tcp.split(","))]
    left_chain, right_chain = chain_for("l"), chain_for("r")
    return {
        "base_frame": base,
        "left": {"side": "left", "joints": left_chain,
                 "tip_link": tip_link_left or find_tip_link(left_chain),
                 "tcp_offset": tcp},
        "right": {"side": "right", "joints": right_chain,
                  "tip_link": tip_link_right or find_tip_link(right_chain),
                  "tcp_offset": tcp},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None,
                    help="путь к URDF (по умолчанию ищется tiangong_pro_with_hands.urdf)")
    ap.add_argument("--out", default="../config/arms.json")
    ap.add_argument("--base", default="pelvis")
    ap.add_argument("--tcp-offset", default="0.05,0.0,0.0",
                    help="смещение TCP от frame запястья x,y,z (подобрать по кончику пальцев)")
    ap.add_argument("--tip-link-left", default=None,
                    help="имя link-а TCP слева (по умолчанию — лист fixed-цепочки от wrist_roll_l)")
    ap.add_argument("--tip-link-right", default=None)
    args = ap.parse_args()

    urdf = find_urdf(args.urdf)
    data = build_config(urdf, base=args.base, tcp=args.tcp_offset,
                        tip_link_left=args.tip_link_left,
                        tip_link_right=args.tip_link_right)
    import os
    d = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(d, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"OK: записано {args.out} (цепочки l/r по 7 суставов из {urdf})")


if __name__ == "__main__":
    main()
