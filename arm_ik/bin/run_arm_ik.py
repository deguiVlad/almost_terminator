#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Запуск ЕДИНСТВЕННОЙ ноды пакета arm_ik (IK обеих рук, 7 DOF на руку).

Пакет сведён к одной ноде `arm_ik_node`:
  * вход: /arm_ik/goal_{left,right}  (PoseStamped — абсолютная 6D-поза TCP),
          /arm_ik/delta_{left,right} (TwistStamped — 6D-смещение);
  * выход: /arm/cmd_pos (bodyctrl_msgs), телеметрия позы и статуса;
  * опционально сама публикует демо-цели: -p demo_goals:=true или
    -p demo_delta:=true — отдельные демо-ноды не нужны.

Работает из исходников (./bin/run_arm_ik.py), из install/lib/arm_ik/ и через
`ros2 run arm_ik run_arm_ik.py`. Имя файла намеренно НЕ совпадает с именем
модуля (arm_ik/nodes/arm_ik_node.py), чтобы исключить circular import.
Детали авто-нахождения python с rclpy — в bin/_bootstrap.py.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import bootstrap  # noqa: E402

if __name__ == "__main__":
    bootstrap("arm_ik_node")()
