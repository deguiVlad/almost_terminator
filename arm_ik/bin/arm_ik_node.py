#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone launcher: works without colcon build (sys.path bootstrap).
Usage: ./bin/arm_ik_node.py  OR  ros2 run via this script directly."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_ik.arm_ik_node import main
if __name__ == "__main__":
    main()
