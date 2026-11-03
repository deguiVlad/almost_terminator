# 📋 Шпаргалка по ROS 2-интерфейсам хакатона (nssr-sim)

Быстрый справочник по командам. Полное описание интерфейсов, моторов и задач — в [INTERFACES.md](INTERFACES.md),
условия задач — в [TASKS.md](TASKS.md), оценка — в [SCORING.md](SCORING.md).

Симулятор `nssr-sim` — контейнер с ROS 2 Humble + Isaac Sim. `ROS_DOMAIN_ID=146`, профиль DDS
`hackathon/docker/fastdds_hack.xml`. Обёртка `bash hackathon/tools/ros.sh <команда>` сама источникит окружение
ROS и подставляет домен; без неё перед любой `ros2`-командой выполняйте
`source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=146`.

## 1. Работа с симуляцией

| Действие | Команда |
|---|---|
| Список топиков / узлов | `bash hackathon/tools/ros.sh ros2 topic list` (аналогично `ros2 node list`) |
| Проверить интерфейсы | `bash hackathon/tools/ros.sh python3 /nssr_sim/hackathon/tools/check_interfaces.py` |
| Сборка и запуск hello_world | `bash hackathon/tools/ros.sh "cd /nssr_sim/hackathon/ros2 && colcon build --packages-select hello_world && source install/setup.bash && ros2 run hello_world hello"` |

## 2. Узлы симулятора

| Узел | Назначение |
|---|---|
| `/hack_sim_node` | ядро симулятора: физика и состояние мира |
| `/cloud_cam_head`, `/cloud_cam_torso` | драйверы камер головы и торса (RGB, depth, облака точек) |
| `/hack_camera_bridge` | бридж ROS 2 ↔ внутренний SDK камер |
| `/tiangong_ros_bridge` | бридж совместимости с роботом TienKung |
| `/robot_state_publisher` | дерево TF из URDF и `/joint_states` |
| `/rviz` | визуализация (запускается отдельно, `rviz2`) |
| `/transform_listener_impl_*` | внутренние слушатели TF2 |

## 3. Топики

Направление: **в** — команды роботу, **из** — данные от робота.

### 3.1 Движение (arm / head / waist)

Для каждой группы `{arm,head,waist}` свои топики; ID моторов и пределы суставов — в [INTERFACES.md](INTERFACES.md).

| Топик | Тип | Напр. | Частота | Примечание |
|---|---|---|---|---|
| `/{arm,head,waist}/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | в | — | позиционное управление: `name` (ID), `pos` (рад), `spd` (рад/с, 0 = с предельной скоростью), `cur` не используется |
| `/{arm,head,waist}/cmd_vel` | `bodyctrl_msgs/CmdSetMotorSpeed` | в | — | скоростное управление, рад/с; без новых команд 0.25 с мотор держит позицию |
| `/{arm,head,waist}/cmd_ctrl` | `bodyctrl_msgs/CmdMotorCtrl` | в | — | PID-набор полей (kp, kd, pos, spd, tor); в симуляторе работает только `pos` |
| `/{arm,head,waist}/status` | `bodyctrl_msgs/MotorStatusMsg` | из | 20 Гц | `pos` и `speed` каждого мотора |
| `/leg/*` | те же типы | — | — | заморожены: ног в симуляции нет, не тратьте время |

Скорость всех моторов рук, головы и корпуса ограничена 0.8 рад/с (как у реального робота на хакатоне).

### 3.2 Кисти Inspire

| Топик | Тип | Напр. | Частота |
|---|---|---|---|
| `/inspire_hand/ctrl/{left,right}_hand` | `sensor_msgs/JointState` | в | — |
| `/inspire_hand/state/{left,right}_hand` | `sensor_msgs/JointState` | из | 20 Гц |

Пальцы `name`: `1` мизинец, `2` безымянный, `3` средний, `4` указательный, `5` сгиб большого пальца,
`6` поворот большого пальца. `position`: `1.0` раскрыт, `0.0` сжат.

### 3.3 Камеры (`cam_head`, `cam_torso`)

| Топик | Тип | Частота |
|---|---|---|
| `/cam_*/color/image_raw` | `sensor_msgs/Image` (`rgb8`) | ~5 Гц |
| `/cam_*/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, мм; 0 = нет данных) | ~5 Гц |
| `/cam_*/{color,depth}/camera_info` | `sensor_msgs/CameraInfo` | ~5 Гц |
| `/cam_*/points` | облако точек | ~5 Гц |

Камеры можно ограничить одной головой: `--cams head` / `HACK_CAMS` — тогда топиков `cam_torso` нет, проверяйте их
наличие. Разрешение 640x360, горизонтальный угол обзора 86°.

### 3.4 Сенсоры и системные

| Топик | Тип | Частота | Примечание |
|---|---|---|---|
| `/arm_6dof_{left,right}` | `geometry_msgs/WrenchStamped` | 20 Гц | нагрузка на кисть в рамке `wrist_roll_*_link`, вес кисти вычтен; пустая кисть ≈ 0 |
| `/joint_states` | `sensor_msgs/JointState` | 20 Гц | все суставы, включая пальцы |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 10 Гц / стат. | дерево звеньев из URDF |
| `/clock` | `rosgraph_msgs/Clock` | 20 Гц | ⚠️ время симуляции: узлы запускайте с `use_sim_time:=true`, не используйте системное время |
| `/robot_description` | `std_msgs/String` | — | URDF робота |

### 3.5 Топики задачи

| Топик | Тип | Напр. | Примечание |
|---|---|---|---|
| `/sim/task` | `std_msgs/String`, JSON | из, latched | задача и правило; читать при старте |
| `/sim/objects/names` | `std_msgs/String`, JSON | из, latched | объекты и категории, без координат |
| `/sim/task/done` | `std_msgs/Empty` | в | сигнал завершения раскладки, см. [SCORING.md](SCORING.md) |
| `/sim/task/check` → `/sim/task/result` | `std_msgs/Int32` → `std_msgs/String`, JSON | в → из | самопроверка, отладка; на итоговой проверке отключён |
| `/sim/objects/randomize` | `std_msgs/Int32` | в | новая раскладка по сидy, отладка |
| `/sim/objects/set` | `std_msgs/String`, JSON | в | поставить объект `{"имя": [x, y, yaw]}`, отладка |

Latched = durability `TRANSIENT_LOCAL`: подписчик получает последнее сообщение сразу после подписки.
Форматы JSON и правила задач — в [INTERFACES.md](INTERFACES.md) и [TASKS.md](TASKS.md).

## 4. Сервисы кистей

| Сервис | Назначение |
|---|---|
| `/inspire_hand/set_angle_flexible/{left,right}_hand` | то же, что `ctrl/{...}_hand`, только сервисом |
| `/inspire_hand/set_speed/{left,right}_hand` | скорость пальцев: 1.0 = полный ход за 0.8 с (по умолчанию), 0.5 = за 1.6 с |
| `/inspire_hand/set_force/{left,right}_hand` | предел силы: 1.0 соответствует 1000 г (по умолчанию) |
| `/inspire_hand/set_clear_error/{left,right}_hand` | всегда успешен: ошибок кисти в симуляторе нет |

У остальных узлов (`/hack_sim_node`, драйверы камер, `/robot_state_publisher` и т.д.) — стандартные сервисы
параметров ROS 2: `get_parameters`, `set_parameters`, `list_parameters` и др.

## 5. Быстрые примеры

```bash
# Повернуть голову влево (head_yaw, мотор 3) на 0.5 рад со скоростью 0.5 рад/с
bash hackathon/tools/ros.sh "ros2 topic pub -t 3 /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition \
  '{cmds: [{name: 3, pos: 0.5, spd: 0.5, cur: 1.0}]}'"

# Сжать правую кисть (позиция 0.2 = почти кулак, большой палец 0.5)
bash hackathon/tools/ros.sh "ros2 topic pub -t 3 /inspire_hand/ctrl/right_hand sensor_msgs/msg/JointState \
  \"{name: ['1','2','3','4','5','6'], position: [0.2, 0.2, 0.2, 0.2, 0.5, 0.2]}\""

# Посмотреть задачу, нагрузку на запястье и калибровку камеры
bash hackathon/tools/ros.sh "ros2 topic echo --once /sim/task"
bash hackathon/tools/ros.sh ros2 topic echo --once /arm_6dof_right
bash hackathon/tools/ros.sh ros2 topic echo --once /cam_head/color/camera_info
```

## 6. Подводные камни

1. **Время симуляции.** Симулятор идёт медленнее реального времени; метки в `/clock`. Без `use_sim_time:=true`
   таймеры и ожидания вашего кода будут считаться в реальном времени и «убегут» от симуляции.
2. **Ноги.** Топики `/leg/*` есть для совместимости с реальным роботом, но заморожены; корпус поворачивается
   только мотором 31 (`waist`). Наклон и высота базы (`/hric/robot/float_base_rpyz_cmd`) — только на реальном роботе.
3. **Камеры.** В режиме `--cams head` торсовая камера отсутствует; перед использованием проверяйте наличие топиков.
4. **Ограничение скорости 0.8 рад/с** действует и на итоговой проверке — решения «в обход» лимита не пройдут.
