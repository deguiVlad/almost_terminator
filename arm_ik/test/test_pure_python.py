# -*- coding: utf-8 -*-
"""Тест pure-python fallback ядра arm_ik (без numpy/scipy).

Запускает весь офлайн-тестовый набор math-ядра в subprocess с
ARM_IK_FORCE_PURE=1, где _math_compat подменяет numpy/scipy чистым
Python-слоем (_pure_np). Это гарантия, что пакет работает на машинах без
pip-зависимостей (сломанные venv/оверлеи на роботе).
"""
import os
import subprocess
import sys

import pytest


@pytest.mark.skipif(sys.version_info < (3, 8), reason="subprocess env")
def test_core_tests_pass_in_pure_python_mode():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ, ARM_IK_FORCE_PURE="1")
    r = subprocess.run(
        [sys.executable, "-m", "pytest", "test/test_kinematics.py", "test/test_tree_from_tf.py", "test/test_urdf_model.py", "test/test_collision.py", "-q"],
        cwd=root, env=env, capture_output=True, text=True)
    assert r.returncode == 0, (
        f"pure-python режим упал:\nSTDOUT:\n{r.stdout[-4000:]}\n"
        f"STDERR:\n{r.stderr[-2000:]}")
