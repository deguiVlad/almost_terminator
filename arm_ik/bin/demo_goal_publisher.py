#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone launcher: works without colcon build (sys.path bootstrap).
Usage: ./bin/demo_goal_publisher.py  OR  ros2 run via this script directly."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_ik.nodes.demo_goal_publisher import main
if __name__ == "__main__":
    main()
