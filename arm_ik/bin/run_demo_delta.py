#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Демо delta-целей (TwistStamped). Имя не совпадает с модулем — см. _bootstrap."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import bootstrap  # noqa: E402

if __name__ == "__main__":
    bootstrap("demo_delta_publisher")()
