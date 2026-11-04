#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Запуск IK-сервера обеих рук. Работает и из исходников, и из install/lib/arm_ik.

Имя файла намеренно НЕ совпадает с именем модуля (arm_ik_node.py), чтобы
исключить ImportError: circular import, который возникал у старых обёрток.
Детали — в bin/_bootstrap.py.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import bootstrap  # noqa: E402

if __name__ == "__main__":
    bootstrap("arm_ik_node")()
