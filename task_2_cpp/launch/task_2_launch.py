#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ament_python_launch import LaunchDescription
from ament_cmake_node import AmentNodeDescription, CmdLaunchArgument
import os

def main():
    launch_description = LaunchDescription()
    
    # Добавляем амент_cmake ноду
    node_name = 'zahvat_node'
    exec_depends_on = []
    cmd_args = ['rcl']  # Запуск ROS2
    
    # Создаём амент_cmake ноду для запуска zahvat.cpp
    launch_description.add_node(
        pkg='task_2_cpp',
        node_name=node_name,
        depends=exec_depends_on,
        executable='/home/user/nssr-polygon/hackathon/ros2/task_2_cpp/bin/zahvat_node'
    )
    
    # Добавляем запуск ркл для всех зависимых нод
    launch_description.add_action(AmentNodeDescription(pkg='task_2_cpp', node_name=node_name))
    
    # Запускаем командную строку с аргументами (если нужны)
    # cmd_args.append('--ros-args')
    # cmd_args.extend(['--use-intrinsic-types'])
    
    return launch_description

if __name__ == '__main__':
    import sys
    main()