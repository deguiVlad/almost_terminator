# focus_tracking

Система фокусировки камеры головы на **объектах**, на **собственных руках**
робота или на **обоих сразу**. Полный расчёт математики и обоснование архитектуры —
в [docs/FOCUS_TRACKING.md](../docs/FOCUS_TRACKING.md).

## Ноды

| Исполняемый файл | Язык | Роль |
|---|---|---|
| `object_detector_node.py` | Python (rclpy, OpenCV) | HSV-детекция blob + глубина → `/focus/target` (PoseStamped, `camera_head_link`) |
| `hand_tracker_node.cpp` | C++ (rclcpp, TF2) | позиция запястья из кинематики → `/focus/hand`, `/focus/hand_visible` |
| `focus_controller_node.cpp` | C++ (rclcpp) | цель(и) → углы yaw/pitch → `/head/cmd_pos`; параметры `source:=object\|hand\|both` |

Единый контракт: все цели — `geometry_msgs/PoseStamped` в рамке `camera_head_link`.
Контроллер не различает источник — новый объект слежения = новая нода-паблишер.

## Сборка и запуск

```bash
colcon build --packages-select focus_tracking
source install/setup.bash
ros2 launch focus_tracking focus.launch.py source:=both hand:=right
```

По отдельности:

```bash
ros2 run focus_tracking focus_controller_node --ros-args -p use_sim_time:=true -p source:=hand
ros2 run focus_tracking hand_tracker_node     --ros-args -p use_sim_time:=true -p hand:=left
ros2 run focus_tracking object_detector_node.py --ros-args -p use_sim_time:=true \
  -p "target_hsv:=[170,100,100,10,255,255]"   # красный blob
```

Управление на лету:

```bash
ros2 service call /focus/set_mode std_srvs/srv/SetBool "{data: true}"    # ACTIVE / false = IDLE
ros2 service call /focus/set_hand std_srvs/srv/SetBool "{data: true}"    # true=правая, false=левая
```

Параметры (deadzone, веса both-режима, лимиты, HSV-окно) — в
[config/focus_params.yaml](config/focus_params.yaml). Требуется `bodyctrl_msgs`
из контейнера симулятора и работающий `robot_state_publisher` (в симе запущен).

## Проверка без симулятора

```bash
# сгенерировать цель руками:
ros2 topic pub -r 10 /focus/target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: camera_head_link}, pose: {position: {x: 0.1, y: 0.0, z: 0.8}}}"
ros2 topic echo /head/cmd_pos
```
