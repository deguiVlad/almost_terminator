#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Экспорт кинематики рук из URDF робота в config/arms.json.

Запуск внутри контейнера симулятора (или где есть доступ к файлам проекта):

    # 1) из файла URDF:
    python3 export_kinematics_from_urdf.py --urdf /nssr_sim/assets/robots/tiangong_pro/robot.urdf \
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


def parse_origin(el):
    if el is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in (el.get("xyz", "0 0 0").split())]
    rpy = [float(v) for v in (el.get("rpy", "0 0 0").split())]
    return xyz, rpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True, help="путь к robot.urdf")
    ap.add_argument("--out", default="../config/arms.json")
    ap.add_argument("--base", default="pelvis")
    ap.add_argument("--tcp-offset", default="0.05,0.0,0.0",
                    help="смещение TCP от frame запястья x,y,z (подобрать по кончику пальцев)")
    args = ap.parse_args()

    tree = ET.parse(args.urdf)
    root = tree.getroot()

    joints = {}
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        axis_el = j.find("axis")
        axis = [float(v) for v in axis_el.get("xyz", "0 0 1").split()] if axis_el is not None else [0, 0, 1]
        lim_el = j.find("limit")
        lo = float(lim_el.get("lower", "-3.14")) if lim_el is not None else -3.14
        up = float(lim_el.get("upper", "3.14")) if lim_el is not None else 3.14
        xyz, rpy = parse_origin(j.find("origin"))
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            axis=axis, origin_xyz=xyz, origin_rpy=rpy, limits=[lo, up])

    def chain_for(tag):
        names = [f"shoulder_pitch_{tag}_joint", f"shoulder_roll_{tag}_joint",
                 f"shoulder_yaw_{tag}_joint", f"elbow_pitch_{tag}_joint",
                 f"elbow_yaw_{tag}_joint", f"wrist_pitch_{tag}_joint",
                 f"wrist_roll_{tag}_joint"]
        out = []
        for n in names:
            if n not in joints:
                raise SystemExit(f"В URDF нет сустава {n}. Проверьте имена звеньев в {args.urdf}")
            jd = dict(joints[n])
            jd["name"] = n
            jd["motor_id"] = MOTOR_IDS[n]
            out.append(jd)
        return out

    tcp = [float(v) for v in args.tcp_offset.split(",")]
    data = {
        "base_frame": args.base,
        "left": {"side": "left", "joints": chain_for("l"),
                 "tip_link": "wrist_roll_l_link", "tcp_offset": tcp},
        "right": {"side": "right", "joints": chain_for("r"),
                  "tip_link": "wrist_roll_r_link", "tcp_offset": tcp},
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"OK: записано {args.out} (цепочки l/r по 7 суставов из {args.urdf})")


if __name__ == "__main__":
    main()
