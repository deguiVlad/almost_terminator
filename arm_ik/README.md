# arm_ik — обратная кинематика двух рук TienKung 2.0 Pro

ROS 2 Humble Python-пакет для двух 7-DOF рук. Кинематическая модель и пределы
суставов читаются из URDF. Узел решает IK для TCP кисти, строит ограниченную по
скорости траекторию и публикует команды моторов через `bodyctrl_msgs`.

> **Важно:** перед реальным движением проверьте URDF, motor ID mapping, направления
> осей, единицы и ограничения на стенде без нагрузки. Встроенная проверка стола
> является упрощённой геометрической защитой, а не заменой MoveIt/FCL и
> полноценной проверки коллизий. Не подключайте выход к приводам до проверки.

## 1. Требования

- Ubuntu 22.04 и ROS 2 Humble;
- `rclpy`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`, `std_msgs`;
- `bodyctrl_msgs` из окружения робота для публикации `/arm/cmd_pos`;
- `numpy` и `scipy` рекомендуются. Для математического ядра есть pure-Python
  fallback, но ROS 2 и драйвер сообщений всё равно должны быть установлены.

## 2. Сборка

Из корня ROS workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select arm_ik --symlink-install
source install/setup.bash
```

Проверить установленный исполняемый файл:

```bash
ros2 pkg prefix arm_ik
ros2 pkg executables arm_ik
```

Пакет устанавливает исполняемый файл `run_arm_ik.py`. Вызов `ros2 run arm_ik arm_ik`
не предусмотрен.

## 3. Запуск

Сначала запустите robot_state_publisher и источник `/joint_states`. URDF можно
передать параметром `urdf_path`:

```bash
ros2 launch arm_ik arm_ik.launch.py \
  urdf_path:=/absolute/path/to/tiangong2pro.urdf
```

Для реального робота wall time используется по умолчанию. В симуляции явно
включите ROS simulation time: `use_sim_time:=true`.

Либо оставьте `urdf_path` пустым: узел ожидает transient-local топик
`/robot_description` типа `std_msgs/msg/String`:

```bash
ros2 launch arm_ik arm_ik.launch.py urdf_path:=""
```

В модели должны существовать цепи от `waist_yaw_link` до `left_tcp_link` и
`right_tcp_link`, каждая с семью вращательными суставами в порядке:

1. `shoulder_pitch_{l,r}_joint`
2. `shoulder_roll_{l,r}_joint`
3. `shoulder_yaw_{l,r}_joint`
4. `elbow_pitch_{l,r}_joint`
5. `elbow_yaw_{l,r}_joint`
6. `wrist_pitch_{l,r}_joint`
7. `wrist_roll_{l,r}_joint`

Модель строится только из URDF: TF не содержит сведений об исходных осях и
пределах суставов, поэтому TF-only построение исполнительной модели отключено.
TF2 используется для преобразования входных целевых поз между системами координат.

## 4. ROS интерфейсы

### Абсолютная цель

- `/arm_ik/goal_left`
- `/arm_ik/goal_right`
- тип: `geometry_msgs/msg/PoseStamped`

Поза TCP (метры + quaternion). `header.frame_id` должен существовать в TF.
Если timestamp ненулевой, преобразование выполняется на соответствующий момент
времени; нулевой timestamp означает latest transform.

Пример:

```bash
ros2 topic pub --once /arm_ik/goal_right geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: waist_yaw_link}, pose: {position: {x: 0.40, y: -0.25, z: 0.95}, orientation: {w: 1.0}}}"
```

### Относительная цель

- `/arm_ik/delta_left`
- `/arm_ik/delta_right`
- тип: `geometry_msgs/msg/TwistStamped`

`twist.linear` — смещение TCP в метрах; `twist.angular` — rotation vector
(ось × угол, радианы), а не угловая скорость. По умолчанию оба вектора заданы
в базовой системе координат. При `delta_rotation_frame:=tcp` вектор вращения
задаётся в локальных осях TCP; для этого режима `header.frame_id` должен быть
пустым или равным `base_frame`.

Параметр `delta_reference`:
- `measured` — опорная поза вычисляется по текущим измеренным суставам;
- `last_goal` — опорная поза берётся из последней принятой цели.

`delta_accumulate` включает накопление последовательных дельт от последней
цели, пока рука находится в активном или достигнутом состоянии.

### Выходы

| Топик | Тип | Назначение |
|---|---|---|
| `/arm/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | Команды моторным ID 11–17 и 21–27 |
| `/arm_ik/wrist_left_pose` | `geometry_msgs/msg/PoseStamped` | Измеренная FK-поза левого TCP |
| `/arm_ik/wrist_right_pose` | `geometry_msgs/msg/PoseStamped` | Измеренная FK-поза правого TCP |
| `/arm_ik/left/status` | `std_msgs/msg/String` | JSON-состояние левой руки |
| `/arm_ik/right/status` | `std_msgs/msg/String` | JSON-состояние правой руки |

Если `bodyctrl_msgs` недоступен, узел работает в dry-run: IK и статусы
рассчитываются, но команды в `/arm/cmd_pos` не публикуются.

## 5. Защита и ограничения

- Жёсткий верхний предел скорости команд — 0.8 рад/с; `spd_limit` может только
  уменьшить его.
- `joint_state_timeout` — watchdog свежести обратной связи. При отсутствии
  или устаревании `/joint_states` публикация команд приостанавливается.
- `enable_table_guard:=true` включает упрощённую проверку столешницы. Её
  координаты `h_table`, `table_x0`, `table_x1`, `table_y_half` должны быть
  откалиброваны относительно `base_frame`.
- Точки вдоль сегментов звеньев дискретно проверяются с учётом условного радиуса.
  Меши, корпус, вторая рука и внешние объекты не проверяются.
- Если прямая суставная траектория пересекает стол, узел пытается построить
  поднятый waypoint и проверяет обе части маршрута. Если безопасный маршрут
  не найден, цель отклоняется.
- IK — численный Damped Least Squares solver. Недостижимые цели отклоняются;
  гарантии нахождения глобального решения нет.

## 6. Параметры запуска

| Параметр | По умолчанию | Значение |
|---|---:|---|
| `urdf_path` | пусто | Путь к URDF; иначе используется `/robot_description` |
| `use_sim_time` | false | Включать ROS time из `/clock` для симуляции |
| `base_frame` | `waist_yaw_link` | Система координат целей |
| `kinematics_base_frame` | `waist_yaw_link` | Корень URDF-цепей |
| `rate_hz` | 50 | Частота control loop |
| `joint_state_timeout` | 0.5 с | Допустимый возраст обратной связи |
| `spd_limit` | 0.8 рад/с | Верхняя граница скорости |
| `a_max` | 0.4 рад/с² | Ускорение профиля |
| `tol_pos` | 0.01 м | Допуск по позиции |
| `tol_rot` | 0.05 рад | Допуск по ориентации |
| `enable_table_guard` | false | Включение проверки стола |
| `waypoint_lift` | 0.25 м | Высота обходного waypoint |
| `tcp_offset` | пусто | Опциональный override TCP offset из URDF: `x,y,z` в метрах |
| `delta_reference` | `measured` | Опора delta-команд |
| `delta_accumulate` | true | Накопление delta-команд |
| `delta_rotation_frame` | `base` | Оси rotation vector: `base` или `tcp` |

Для полного списка параметров и demo-аргументов смотрите
`launch/arm_ik.launch.py` и объявления параметров в
`arm_ik/nodes/arm_ik_node.py`.

## 7. Встроенное демо

```bash
ros2 launch arm_ik arm_ik.launch.py demo:=true
```

Демо публикует абсолютные цели. Для дельт:

```bash
ros2 launch arm_ik arm_ik.launch.py demo_delta:=true delta_side:=right delta_x:=0.03
```

Не запускайте demo на подключённых приводах без предварительной проверки
рабочей зоны и отключения силового питания.

## 8. Тесты

Офлайн-тесты математического ядра (ROS-нода и реальные моторы не требуются):

```bash
cd <workspace>/src/arm_ik
python3 -m pytest test -q
ARM_IK_FORCE_PURE=1 python3 -m pytest test -q
```

Проверки на ROS 2 Humble и на реальном роботе должны выполняться отдельно:
сборка пакета, наличие URDF и всех 14 суставов в `/joint_states`, корректность
TF, соответствие motor ID, dry-run, затем стендовая проверка с ограниченной
скоростью и аварийной остановкой.

## 9. Структура

- `arm_ik/kinematics.py` — FK, геометрический Якобиан, DLS IK и delta-позы.
- `arm_ik/urdf_model.py` — извлечение цепей, осей и limits из URDF.
- `arm_ik/motor_mapping.py` — проверяемое сопоставление имён суставов и motor ID.
- `arm_ik/collision.py` — упрощённая геометрическая проверка стола.
- `arm_ik/trajectory.py` — синхронизированный суставной профиль.
- `arm_ik/_math_compat.py`, `arm_ik/_pure_np.py` — адаптер NumPy/SciPy и fallback.
- `arm_ik/nodes/arm_ik_node.py` — ROS 2 узел.
- `launch/arm_ik.launch.py` — запуск узла.
- `bin/run_arm_ik.py`, `bin/_bootstrap.py` — запуск из исходников и install.
- `test/` — офлайн-тесты.
