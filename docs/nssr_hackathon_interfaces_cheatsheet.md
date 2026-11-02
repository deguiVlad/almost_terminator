# 📋 Шпаргалка по ROS2-интерфейсам хакатона (симулятор nssr-sim)

> Автор-концепт: Nikolay Semenov. `nssr-sim` — контейнер с ROS2 Humble + Isaac Sim.
> Домен ROS: `ROS_DOMAIN_ID=146`, профиль DDS: `/nssr_sim/hackathon/docker/fastdds_hack.xml`.
> Для всех `ros2` команд внутри контейнера: сначала `source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=146`.

---

## 1. Как работать с симуляцией

| Действие | Команда |
|---|---|
| Список топиков | `bash ~/nssr-polygon/hackathon/tools/ros.sh ros2 topic list` |
| Проверить интерфейсы | `bash hackathon/tools/ros.sh python3 /nssr_sim/hackathon/tools/check_interfaces.py` |
| Сборка + запуск hello_world | `bash hackathon/tools/ros.sh "cd /nssr_sim/hackathon/ros2 && colcon build --packages-select hello_world && source install/setup.bash && ros2 run hello_world hello"` |

---

## 2. Узлы (nodes)

| Узел | Назначение |
|---|---|
| `/cloud_cam_head` | Камера головы |
| `/cloud_cam_torso` | Камера таза |
| `/hack_camera_bridge` | Бридж ROS2 → внутр. SDK |
| `/hack_sim_node` | Ядро симулятора |
| `/robot_state_publisher` | tf по URDF |
| `/rviz` | Визуализация |
| `/tiangong_ros_bridge` | Тихогонецкий бридж |
| `/transform_listener_impl_*` | listener tf2 |

---

## 3. Топики (subscriptions / publications)

### 3.1 Движение робота (cmd → status)

| Топик | Тип | Направление | Частота |
|---|---|---|---|
| `/arm/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | **в** (pos: rad, spd: rpm, cur: A) | — |
| `/arm/cmd_ctrl` | `bodyctrl_msgs/msg/CmdMotorCtrl` | **в** (PID: kp, kd, pos, spd, tor) | — |
| `/arm/cmd_vel` | `bodyctrl_msgs/msg/CmdSetMotorSpeed` | **в** (spd: rpm, cur: A) | — |
| `/arm/status` | `bodyctrl_msgs/msg/MotorStatusMsg` | **из** (статус моторов) | 20 Hz |
| `/head/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | **в** | — |
| `/head/cmd_ctrl` | `bodyctrl_msgs/msg/CmdMotorCtrl` | **в** | — |
| `/head/cmd_vel` | `bodyctrl_msgs/msg/CmdSetMotorSpeed` | **в** | — |
| `/head/status` | `bodyctrl_msgs/msg/MotorStatusMsg` | **из** | 20 Hz |
| `/waist/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | **в** | — |
| `/waist/cmd_ctrl` | `bodyctrl_msgs/msg/CmdMotorCtrl` | **в** | — |
| `/waist/cmd_vel` | `bodyctrl_msgs/msg/CmdSetMotorSpeed` | **в** | — |
| `/waist/status` | `bodyctrl_msgs/msg/MotorStatusMsg` | **из** | 20 Hz |
| `/leg/cmd_pos`, `/leg/cmd_ctrl`, `/leg/cmd_vel`, `/leg/status` | — | **в/из** | заморожены (legs fixed) |

> `cmd_pos` → позиционный контроль; `cmd_ctrl` → PID + положение + скорость + момент; `cmd_vel` → скоростной (rpm).

### 3.2 Руки / стили / кисти (inspire_hand)

| Топик | Тип | Направление | Частота |
|---|---|---|---|
| `/inspire_hand/state/left_hand` | `sensor_msgs/msg/JointState` | **из** | 20 Hz |
| `/inspire_hand/state/right_hand` | `sensor_msgs/msg/JointState` | **из** | 20 Hz |
| `/inspire_hand/ctrl/left_hand` | `sensor_msgs/msg/JointState` | **в** (name 1–6, position 0–1) | — |
| `/inspire_hand/ctrl/right_hand` | `sensor_msgs/msg/JointState` | **в** (name 1–6, position 0–1) | — |

> На каждый палец: `1 little, 2 ring, 3 middle, 4 index, 5 thumb_2, 6 thumb_1`. `position 0` = закрыто, `1` = открыто.

### 3.3 Камеры

| Топик | Тип | Частота |
|---|---|---|
| `/cam_head/color/image_raw` | `sensor_msgs/msg/Image` | 5 Hz |
| `/cam_head/depth/image_raw` | `sensor_msgs/msg/Image` | 5 Hz |
| `/cam_head/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 5 Hz |
| `/cam_head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 5 Hz |
| `/cam_head/points` | (point cloud) | — |
| `/cam_torso/color/image_raw` | `sensor_msgs/msg/Image` | 5 Hz |
| `/cam_torso/depth/image_raw` | `sensor_msgs/msg/Image` | 5 Hz |
| `/cam_torso/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 5 Hz |
| `/cam_torso/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 5 Hz |
| `/cam_torso/points` | (point cloud) | — |

### 3.4 Ощущения и общие

| Топик | Тип | Частота |
|---|---|---|
| `/arm_6dof_left`, `/arm_6dof_right` | `geometry_msgs/msg/WrenchStamped` | 20 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | 20 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | 10 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | статичный |
| `/clock` | `rosgraph_msgs/msg/Clock` | 20 Hz |
| `/robot_description` | `std_msgs/msg/String` (URDF) | — |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | — |
| `/rosout` | `rcl_interfaces/msg/Log` | — |
| `/sim/objects/names` | `std_msgs/msg/String` (JSON) | latched |
| `/sim/task` | `std_msgs/msg/String` (JSON) | latched |
| `/sim/task_result` | `std_msgs/msg/String` (JSON) | latched |
| `/sim/task_completed` | — | when done |
| `/sim/cmd_reset` | — | reset |
| `/scene/apple/offset` | `std_msgs/msg/String` (JSON) | latched |

> Симуляция работает в **SIM TIME**. `check_interfaces.py` ожидает `t = sec + nanosec*1e-9`.

---

## 4. Сервисы

### 4.1 Руки inspire_hand

| Сервис | Для | Назначение |
|---|---|---|
| `/inspire_hand/set_angle_flexible/left_hand` | left | Компенсация угла (flex) |
| `/inspire_hand/set_angle_flexible/right_hand` | right | Компенсация угла (flex) |
| `/inspire_hand/set_speed/left_hand` | left | Скорость (speed1..6_ratio) |
| `/inspire_hand/set_speed/right_hand` | right | Скорость (speed1..6_ratio) |
| `/inspire_hand/set_force/left_hand` | left | Сила (force1..6) |
| `/inspire_hand/set_force/right_hand` | right | Сила (force1..6) |
| `/inspire_hand/set_clear_error/left_hand` | left | Сброс ошибок |
| `/inspire_hand/set_clear_error/right_hand` | right | Сброс ошибок |

> Тип `bodyctrl_msgs/srv/SetSpeed` → поля `speed0_ratio`..`speed5_ratio` (0..1).

### 4.2 Параметрические сервисы (камеры, бриджи, робот)
`/cloud_cam_head/*`, `/cloud_cam_torso/*`, `/hack_camera_bridge/*`, `/hack_sim_node/*`,
`/robot_state_publisher/*`, `/rviz/*`, `/tiangong_ros_bridge/*` — наборы `describe_parameters`, `get_parameter_types`, `get_parameters`, `list_parameters`, `set_parameters`, `set_parameters_atomically`.

---

## 5. Устройство робота (модульная структура)

```text
[Камера головы] cam_head ── RGB + depth + точечная
[Камера таза]   cam_torso ── RGB + depth + точечная

[Голова] head (3 степени):  head_roll, head_pitch, head_yaw (motor 1-3)
[Пояс]   waist (1 степень): body_yaw (motor 31)

[Левый плечо]  arm (7 степеней) shoulder_pitch_l, shoulder_roll_l, shoulder_yaw_l,
                 elbow_pitch_l, elbow_yaw_l, wrist_pitch_l, wrist_roll_l (motor 11-17)
[Правый плечо] arm (7 степеней) shoulder_pitch_r, shoulder_roll_r, shoulder_yaw_r,
                 elbow_pitch_r, elbow_yaw_r, wrist_pitch_r, wrist_roll_r (motor 21-27)

[Левая кисть] left  — 6 пальцев (little_1, ring_1, middle_1, index_1, thumb_2, thumb_1)
[Правая кисть] right — 6 пальцев (same order)

[Ноги] leg (5 степеней каждая) — в симуляции заморожены
```

### Справочная таблица моторов (id → joint → группа)

| ID | Joint | Группа | Справочная скорость |
|---|---|---|---|
| 1 | head_roll_joint | head | 0.20 |
| 2 | head_pitch_joint | head | 0.20 |
| 3 | head_yaw_joint | head | 0.30 |
| 31 | body_yaw_joint | waist | 0.25 |
| 11 | shoulder_pitch_l_joint | arm | 0.20 |
| 12 | shoulder_roll_l_joint | arm | 0.20 |
| 13 | shoulder_yaw_l_joint | arm | 0.30 |
| 14 | elbow_pitch_l_joint | arm | -0.30 |
| 15 | elbow_yaw_l_joint | arm | 0.30 |
| 16 | wrist_pitch_l_joint | arm | 0.20 |
| 17 | wrist_roll_l_joint | arm | 0.30 |
| 21 | shoulder_pitch_r_joint | arm | 0.20 |
| 22 | shoulder_roll_r_joint | arm | -0.20 |
| 23 | shoulder_yaw_r_joint | arm | -0.30 |
| 24 | elbow_pitch_r_joint | arm | -0.30 |
| 25 | elbow_yaw_r_joint | arm | -0.30 |
| 26 | wrist_pitch_r_joint | arm | 0.20 |
| 27 | wrist_roll_r_joint | arm | -0.30 |

> `bodyctrl_msgs/msg/MotorName` определяет эти идентификаторы (00 = head, 10 = arm left, 20 = arm right, 30 = body, 50 = leg left, 60 = leg right).

### Кисти (inspire_hand)

| Номер | Имя_joint | Управление через `/inspire_hand/ctrl/{left,right}_hand` (JointState, pos 0–1) |
|---|---|---|
| 1 | little_1_joint | 0 закрыто, 1 открыто |
| 2 | ring_1_joint | — |
| 3 | middle_1_joint | — |
| 4 | index_1_joint | — |
| 5 | thumb_2_joint | — |
| 6 | thumb_1_joint | — |

### Определение forces/torques

- `/arm_6dof_left`, `/arm_6dof_right` — `WrenchStamped` в рамке `wrist_roll_*_link` (force{x,y,z}, torque{x,y,z}). Пустая кисть → |F| ≈ 0.


---

## 6. Типы сообщений (ключевые)

### `bodyctrl_msgs/msg/MotorStatusMsg`

```text
std_msgs/Header header
  + builtin_interfaces/Time stamp
  + string frame_id
bodyctrl_msgs/MotorStatus[] status
  + uint16 name       # id мотора
  + float32 pos       # rad
  + float32 speed     # rad/s
  + float32 current   # A
  + float32 temperature
  + uint32 error
```

### `bodyctrl_msgs/msg/SetMotorPosition`

```text
uint16 name    # MotorName
float32 pos    # rad
float32 spd    # rpm
float32 cur    # A
```

### `bodyctrl_msgs/msg/MotorCtrl`

```text
uint16 name    # MotorName
float32 kp
float32 kd
float32 pos    # rad
float32 spd    # rpm
float32 tor    # Nm
```

### `sensor_msgs/msg/JointState`

```text
std_msgs/Header header
string[]  name
float64[] position
float64[] velocity
float64[] effort
```

### `geometry_msgs/msg/WrenchStamped`

```text
std_msgs/Header header
geometry_msgs/msg/Wrench wrench
  + Vector3 force  {x,y,z}
  + Vector3 torque {x,y,z}
```

---

## 7. Объекты симуляции и задачи (config в `config/`)

| Конфиг | Назначение |
|---|---|
| `tasks.yaml` | Список заданий: `sort2`, `sort3`, `tubes` |
| `objects.yaml` | Пулы объектов (jar, box, dish, figure, electronics, reserve, sets) + spawn-зона |
| `lab.yaml` | Задание «трубки»: геометрия пробирок, цветов, штативов, варианты (easy\|standard\|color\|lying) |

> Объекты помещаются в sim через `/sim/objects/set` (JSON `{"name": [x, y, yaw]}`).
> Новый расклад — `/sim/objects/randomize` (int SEED). Имена — `/sim/objects/names` (JSON).
> Проверка задачи — `/sim/task/check` (int 1); ответ — `/sim/task_result` (JSON).


---

## 8. hello_world — пример клиентского узла

Структура: `/nssr_sim/hackathon/ros2/hello_world/hello_world/hello.py`.

- Подписка: `/joint_states`, `/cam_head/color/image_raw`, `/cam_head/depth/image_raw`, `/sim/task` (latched)
- Публикация: `/head/cmd_pos` (CmdSetMotorPosition с `name=3` — head_yaw), `/inspire_hand/ctrl/right_hand` (JointState name 1–6, position 0.2–1.0)
- Задача берётся из `/sim/task` JSON: `json.loads(msg.data)`.

> Билд-запуск (как в шпаргалке везде): `colcon build --packages-select hello_world && source install/setup.bash && ros2 run hello_world hello`.

---

## 9. Быстрый старт (cheat)

```bash
# 1. Осмотр
bash ~/nssr-polygon/hackathon/tools/ros.sh ros2 topic list
bash hackathon/tools/ros.sh python3 /nssr_sim/hackathon/tools/check_interfaces.py

# 2. Свой узел (пример: сначала создайте пакет my_pkg)
cd /nssr_sim/hackathon/ros2 && colcon build --packages-select my_pkg && source install/setup.bash
ros2 run my_pkg my_node

# 3. Управление роботом (пример: повернуть голову вправо на 0.3 рад)
#    publish на /head/cmd_pos (CmdSetMotorPosition, array cmds)

# 4. Кисть (закрыть правую на 30%)
rostopic pub /inspire_hand/ctrl/right_hand sensor_msgs/msg/JointState '\
  name: ["1","2","3","4","5","6"] position: [0.3,0.3,0.3,0.3,0.3,0.3]'

# 5. Проверка интерфейсов:
#    bash hackathon/tools/ros.sh python3 /nssr_sim/hackathon/tools/check_interfaces.py
```

---

## 10. Разное (HACK)

| Элемент | Примечание |
|---|---|
| `/hric/robot/float_base_rpyz_cmd` (NOT_IN_SIM) | только реальный робот (тета/поле/высота); в симуляции тазик = yaw 31 |
| `/leg/*` | ног нет в симуляции (legs fixed) |
| `Clock` 20 Hz | симуляция работает в **SIM TIME**; `check_interfaces.py` работает в wall time, но ожидает sim time |
| `torso camera off: HACK_CAMS` | камеры таза могут быть выключены (режим HACK_CAMS) |

---

*Список интерфейсов собран с помощью `ros2 topic list`, `ros2 interface list`, `ros2 service list`, `ros2 node list` в контейнере `nssr-sim` (ROS2 Humble).*

`/robot_state_publisher/*`, `/rviz/*`, `/tiangong_ros_bridge/*` — наборы `describe_parameters`, `get_parameter_types`, `get_parameters`, `list_parameters`, `set_parameters`, `set_parameters_atomically`.
