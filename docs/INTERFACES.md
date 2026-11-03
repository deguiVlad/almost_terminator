# Интерфейсы робота

Топики, типы сообщений и номера моторов в симуляторе такие же, как в SDK реального робота TienKung 2.0 Pro
(ROS 2 Humble). Сообщения робота лежат в пакете `bodyctrl_msgs` (`teleoperation/msgs/*.deb`), в контейнере
симулятора он уже установлен.

- `ROS_DOMAIN_ID=146`, профиль DDS `hackathon/docker/fastdds_hack.xml` (общая память для кадров камер).
- Время симуляции публикуется в `/clock`, все метки времени идут по нему. Узлы запускайте с `use_sim_time:=true`.
- Углы в радианах, скорости в рад/с. Оси робота: x вперёд, y влево, z вверх. Базовое звено `pelvis`.
- Полный формат любого сообщения или сервиса: `bash hackathon/tools/ros.sh ros2 interface show <тип>`, например
  `bodyctrl_msgs/msg/CmdSetMotorPosition`. Пример узла, который читает суставы, камеру и задачу и двигает головой и
  кистью: `hackathon/ros2/hello_world`.
- Краткая справочная таблица всех топиков, узлов, сервисов и частот — в шпаргалке
  [nssr_hackathon_interfaces_cheatsheet_RU.md](nssr_hackathon_interfaces_cheatsheet_RU.md).

## Моторы

| ID | Сустав | Пределы, рад |
|---|---|---|
| 1 | `head_roll_joint` | [-0.45, 0.45] |
| 2 | `head_pitch_joint` (плюс вниз) | [-0.44, 0.44] |
| 3 | `head_yaw_joint` (плюс влево) | [-1.57, 1.57] |
| 31 | `body_yaw_joint`, поворот корпуса (плюс влево) | [-2.97, 3.14] |
| 11 / 21 | `shoulder_pitch_{l,r}_joint` | [-2.97, 2.97] |
| 12 / 22 | `shoulder_roll_{l,r}_joint` | левый [-0.26, 2.62], правый [-2.62, 0.26] |
| 13 / 23 | `shoulder_yaw_{l,r}_joint` | [-2.97, 2.97] |
| 14 / 24 | `elbow_pitch_{l,r}_joint` | [-2.62, 0.26] |
| 15 / 25 | `elbow_yaw_{l,r}_joint` | [-2.97, 2.97] |
| 16 / 26 | `wrist_pitch_{l,r}_joint` | [-0.79, 1.05] |
| 17 / 27 | `wrist_roll_{l,r}_joint` | левый [-1.66, 1.31], правый [-1.31, 1.66] |

Группы: `arm` это моторы 11-17 (левая рука) и 21-27 (правая), `head` это 1-3, `waist` это 31.
Цели за пределами суставов обрезаются по пределам. Пределы взяты из URDF робота; для задачи с пробирками достаточно
повернуть корпус влево на 1.57 рад (90 градусов).

| Топик | Тип | Назначение |
|---|---|---|
| `/{arm,head,waist}/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | `name`: ID мотора, `pos`: цель, `spd`: скорость движения к цели в рад/с (0 значит с предельной скоростью), `cur` не используется |
| `/{arm,head,waist}/cmd_vel` | `bodyctrl_msgs/CmdSetMotorSpeed` | `spd`: скорость в рад/с. Если 0.25 с нет новых команд, мотор останавливается и держит позицию |
| `/{arm,head,waist}/cmd_ctrl` | `bodyctrl_msgs/CmdMotorCtrl` | в симуляторе используется только `pos`, поля kp, kd, spd, tor не эмулируются |
| `/{arm,head,waist}/status` | `bodyctrl_msgs/MotorStatusMsg` | `pos` и `speed` каждого мотора |
| `/joint_states` | `sensor_msgs/JointState` | все суставы робота, включая пальцы |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | дерево звеньев из URDF (`assets/robots/tiangong_pro/`) |

Скорость каждого мотора рук, головы и корпуса ограничена 0.8 рад/с, как у настоящего робота на хакатоне;
ограничение действует и на итоговой проверке. `spd` больше 0.8 и `spd: 0` в `cmd_pos`, а также `cmd_ctrl` дают
0.8 рад/с, `cmd_vel` обрезается до [-0.8, 0.8] рад/с.

Приводы в симуляторе работают как ПД-регуляторы. Под весом руки сустав останавливается немного раньше цели, у
плеча недолёт доходит до 15 %. Для точного позиционирования замыкайте контур по `/joint_states`.

Пример: повернуть голову влево на 0.5 рад со скоростью 0.5 рад/с.

```bash
bash hackathon/tools/ros.sh "ros2 topic pub -t 3 /head/cmd_pos bodyctrl_msgs/msg/CmdSetMotorPosition \
  '{cmds: [{name: 3, pos: 0.5, spd: 0.5, cur: 1.0}]}'"
```

## Кисти

Кисти Inspire, по 6 приводов на кисть. Номера приводов: 1 мизинец, 2 безымянный, 3 средний, 4 указательный,
5 сгиб большого пальца, 6 поворот большого пальца. Положение задаётся долей: 1.0 раскрыт, 0.0 сжат.

| Интерфейс | Тип | Назначение |
|---|---|---|
| `/inspire_hand/ctrl/{left,right}_hand` | `sensor_msgs/JointState` | `name`: "1" ... "6", `position`: цель от 0 до 1 |
| `/inspire_hand/state/{left,right}_hand` | `sensor_msgs/JointState` | `position` от 0 до 1, `velocity` доля максимальной скорости, `effort` доля предела силы |
| `/inspire_hand/set_angle_flexible/{left,right}_hand` | `bodyctrl_msgs/srv/SetAngleFlexible` | то же, что `ctrl`, только сервисом |
| `/inspire_hand/set_speed/{left,right}_hand` | `bodyctrl_msgs/srv/SetSpeed` | скорость пальцев: 1.0 полный ход за 0.8 с (по умолчанию), 0.5 за 1.6 с |
| `/inspire_hand/set_force/{left,right}_hand` | `bodyctrl_msgs/srv/SetForce` | предел силы: 1.0 соответствует 1000 г (по умолчанию) |
| `/inspire_hand/set_clear_error/{left,right}_hand` | `bodyctrl_msgs/srv/SetClearError` | всегда успешно, ошибок кисти в симуляторе нет |

Пример: сжать правую кисть.

```bash
bash hackathon/tools/ros.sh "ros2 topic pub -t 3 /inspire_hand/ctrl/right_hand sensor_msgs/msg/JointState \
  \"{name: ['1','2','3','4','5','6'], position: [0.2, 0.2, 0.2, 0.2, 0.5, 0.2]}\""
```

## Датчики силы в запястьях

`/arm_6dof_left` и `/arm_6dof_right`, тип `geometry_msgs/WrenchStamped`: внешняя нагрузка на кисть, вес самой
кисти уже вычтен. Система координат `wrist_roll_{l,r}_link`. В симуляторе это расчётная величина: направление и
порядок силы и момента как на роботе, точные значения отличаются от настоящего датчика.

```bash
bash hackathon/tools/ros.sh ros2 topic echo --once /arm_6dof_right
```

## Камеры

Две RGB-D камеры: на голове (`cam_head`) и на торсе (`cam_torso`). Разрешение 640x360, горизонтальный угол обзора
86 градусов.

| Топик | Тип |
|---|---|
| `/cam_{head,torso}/color/image_raw` | `sensor_msgs/Image`, `rgb8` |
| `/cam_{head,torso}/depth/image_raw` | `sensor_msgs/Image`, `16UC1`, миллиметры, выровнена по цветной. 0 значит нет данных: ближе 0.15 м или дальше 6 м |
| `/cam_{head,torso}/{color,depth}/camera_info` | `sensor_msgs/CameraInfo` |

Системы координат камер оптические (z вперёд): `camera_head_link` и `camera_body_front_link`. Внутренние параметры
(фокус, центр) в `camera_info`, положение камеры относительно `pelvis` в `/tf`:

```bash
bash hackathon/tools/ros.sh ros2 topic echo --once /cam_head/color/camera_info
bash hackathon/tools/ros.sh ros2 run tf2_ros tf2_echo pelvis camera_head_link
```

Кадры камер в задаче сортировки, голова наклонена вниз на 0.4 рад. Глубина раскрашена для наглядности: красный
ближе, синий дальше, чёрный значит нет данных.

| | Цвет | Глубина |
|---|---|---|
| `cam_head` | ![Головная камера](img/sort3_cam_head.png) | ![Глубина головной камеры](img/sort3_cam_head_depth.png) |
| `cam_torso` | ![Торсовая камера](img/sort3_cam_torso.png) | ![Глубина торсовой камеры](img/sort3_cam_torso_depth.png) |

Пробирки: торсовая камера смотрит на штативы на столе, головная видит штативы в шкафу, когда корпус повёрнут
влево на 1.57 рад.

| | Цвет | Глубина |
|---|---|---|
| `cam_torso`, стол | ![Торсовая камера, штативы на столе](img/tubes_standard_cam_torso.png) | ![Глубина, штативы на столе](img/tubes_standard_cam_torso_depth.png) |
| `cam_head`, шкаф | ![Головная камера, шкаф](img/tubes_hood_cam_head.png) | ![Глубина, шкаф](img/tubes_hood_cam_head_depth.png) |

## Задача

| Топик | Тип | Назначение |
|---|---|---|
| `/sim/task` | `std_msgs/String`, JSON, latched | задача и правило: какая категория в какой ящик, какие пробирки в какие штативы |
| `/sim/objects/names` | `std_msgs/String`, JSON, latched | объекты на столе и их категории |
| `/sim/task/check` | `std_msgs/Int32` | запрос самопроверки, ответ приходит в `/sim/task/result`. Отладка, на итоговой проверке отключён |
| `/sim/task/result` | `std_msgs/String`, JSON | сколько уже на своём месте |
| `/sim/task/done` | `std_msgs/Empty` | решение закончило раскладку; на итоговой проверке по нему останавливается прогон, см. [SCORING.md](SCORING.md) |
| `/sim/objects/randomize` | `std_msgs/Int32` | новая случайная раскладка, число задаёт сид. Отладка, на итоговой проверке отключён |
| `/sim/objects/set` | `std_msgs/String`, JSON | поставить объект: `{"имя": [x, y, yaw]}`, x и y в метрах от таза робота. Отладка, на итоговой проверке отключён |

Latched означает durability `TRANSIENT_LOCAL`: подписчик с такой же QoS получает последнее сообщение сразу после
подписки. Позы объектов не публикуются, находить объекты нужно камерами.

`/sim/task` для сортировки (`objects`: имя объекта и его категория):

```text
{"task": "sort3", "ru": "...", "objects_from": "pool 1",
 "categories": {"jar": "tray_left", "box": "tray_right", "figure": "tray_center"},
 "trays": {"tray_left": "red", "tray_right": "blue", "tray_center": "green"},
 "objects": {"005_tomato_soup_can": "jar", "004_sugar_box": "box", "3D_Dollhouse_Lamp": "figure", ...}}
```

`/sim/task` для пробирок, вариант `standard`. `rule`: `any_slot` в любое гнездо штатива в шкафу, `by_type` в штатив
своего размера (`type` пробирки равен `tube` штатива), `by_cap_color` по цвету крышки (`cap` пробирки равен `color`
штатива). Штативы с `where: hood` стоят в шкафу, с `where: table` на столе:

```text
{"task": "tubes", "variant": "standard", "rule": "by_type", "ru": "...",
 "tubes": {"tube_50_0": {"type": "falcon50", "cap": "blue"}, "tube_15_4": {"type": "falcon15", "cap": "orange"}, ...},
 "racks": {"table_r50_0": {"where": "table", "tube": "falcon50", "color": "white", "slots": 8},
           "table_r15_1": {"where": "table", "tube": "falcon15", "color": "white", "slots": 10},
           "hood_r50_0": {"where": "hood", "tube": "falcon50", "color": "white", "slots": 4},
           "hood_r15_1": {"where": "hood", "tube": "falcon15", "color": "white", "slots": 5}}}
```

`/sim/objects/names`: `{"имя": "категория"}`, для пробирок категория это `тип/цвет крышки`, например
`"tube_50_0": "falcon50/blue"`.

`/sim/task/result` после `check`:

```text
{"in_right_tray": 3, "in_wrong_tray": 1, "not_in_tray": 5, "total": 9, "task": "sort3", "sim_step": 8446,
 "objects": {"005_tomato_soup_can": {"tray": "tray_left", "ok": true}, ...}}
{"placed_ok": 2, "placed_wrong_rack": 0, "not_placed": 2, "total": 4, "task": "tubes", "sim_step": 1563,
 "tubes": {"tube_50_0": {"rack": "hood_r50_0", "upright": true, "ok": true}, ...}}
```

Сигнал завершения. В симуляторе у себя он ни на что не влияет, на итоговой проверке по нему заканчивается прогон:

```bash
bash hackathon/tools/ros.sh "ros2 topic pub --once /sim/task/done std_msgs/msg/Empty '{}'"
```

## Отличия от реального робота

- Ноги закреплены, корпус только поворачивается (мотор 31). Наклон и высота корпуса
  (`/hric/robot/float_base_rpyz_cmd`) есть только на роботе.
- `cmd_ctrl` использует только позицию.
- Силы в запястьях и усилия пальцев расчётные.
- Симулятор работает медленнее реального времени, время берите из `/clock`.
