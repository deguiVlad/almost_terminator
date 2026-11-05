#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Минимальный setup.py без side-effect'ов при разборе.

Эта версия намеренно НЕ содержит параметра `entry_points`: setuptools при
разборе setup.py резолвит все console-script entry points через pkg_resources,
и в сломанных системных окружениях (например, битый pytz/babel из-за
некорректного /etc/localtime) это роняет даже `setup.py --help-commands`,
который вызывает colcon на этапе build. Без entry_points разбор всегда лёгкий;
исполняемый скрипт ставится через setuptools scripts в lib/arm_ik, а ROS-нода
запускаются так:

    ros2 run arm_ik run_arm_ik.py            # или ./bin/run_arm_ik.py напрямую
    ros2 launch arm_ik arm_ik.launch.py

numpy/scipy не объявлены как install_requires — пакет умеет работать на
чистом Python-fallback (arm_ik/_math_compat.py), поэтому сборка проходит и
на машинах без pip-пакетов.
"""
import glob

from setuptools import setup

setup(
    name="arm_ik",
    version="0.4.1",
    packages=["arm_ik", "arm_ik.nodes"],
    # Python-модули устанавливаются setuptools; здесь ставятся ROS-ресурсы,
    # launch-файл и bootstrap рядом с исполняемым скриптом.
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/arm_ik"]),
        ("share/arm_ik", ["package.xml", "README.md", "CHANGELOG.md"]),
        ("share/arm_ik/launch", glob.glob("launch/*.py")),
        # standalone-запуск единственной ноды -> lib/arm_ik/ (script_dir из setup.cfg)
        ("lib/arm_ik", [
            "bin/_bootstrap.py",
            "arm_ik/nodes/arm_ik_node.py"]),
    ],
    scripts=["bin/run_arm_ik.py"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="hackathon team",
    maintainer_email="dev@example.com",
    description="URDF-driven 7-DOF arm IK for TienKung 2.0 Pro",
    license="Apache-2.0",
)
