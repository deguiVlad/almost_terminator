#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone launcher: works without colcon build (sys.path bootstrap).
Re-execs under /usr/bin/python3 if rclpy is not importable (wrong interpreter)."""
import os, sys

def _has_rclpy():
    try:
        import rclpy  # noqa: F401
        return True
    except Exception:
        return False

if not _has_rclpy():
    for py in ("/usr/bin/python3", "/usr/bin/python3.10", "/usr/bin/python3.8"):
        if os.path.exists(py):
            os.execv(py, [py] + sys.argv)
    sys.stderr.write("rclpy not found. Source /opt/ros/humble/setup.bash first.\n")
    sys.exit(1)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in list(sys.path):
    if os.path.basename(p.rstrip("/")) == "arm_ik" and os.path.isdir(p):
        sys.path.remove(p)
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from arm_ik.nodes.demo_delta_publisher import main
if __name__ == "__main__":
    main()
