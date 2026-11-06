# arm_ik — обратная кинематика рук TienKung 2.0 Pro

Выводит кончик каждой 7-DOF руки в заданную позу (позиция + ориентация) относительно `pelvis`.

- Модель руки строится из URDF в `/robot_description`: те же суставы, оси, смещения и пределы, что у
  симулятора. Поворот корпуса (`body_yaw_joint`) берётся из `/joint_states`, так что IK верна и при
  повёрнутом корпусе.
- IK: затухающие наименьшие квадраты (DLS) с обрезкой по пределам суставов; старт из текущей позы, средней
  позы и нескольких случайных. Из подходящих решений выбирается ближайшее к текущей позе.
- Исполнение: одна команда в `/arm/cmd_pos`, скорости суставов согласованы (все приходят одновременно,
  максимум 0.8 рад/с). Когда рука остановилась, нода сравнивает углы из `/joint_states` с решением и
  сдвигает команду на долю ошибки — так компенсируется недолёт ПД-приводов под весом руки.
- Зависимости: только numpy и стандартные пакеты ROS 2 Humble.

## Сборка и запуск

```bash
colcon build --packages-select arm_ik
source install/setup.bash
ros2 launch arm_ik arm_ik.launch.py
```

## Интерфейс

| Топик | Тип | |
|---|---|---|
| `/arm_ik/goal_left`, `/arm_ik/goal_right` | `geometry_msgs/PoseStamped` | цель кончика; рамка любая из TF, пустая = `pelvis` |
| `/arm_ik/left/status`, `/arm_ik/right/status` | `std_msgs/String`, JSON | `idle` / `moving` / `reached` / `stopped` / `unreachable` + `err_pos` (м), `err_rot` (рад) |
| `/arm_ik/wrist_left_pose`, `/arm_ik/wrist_right_pose` | `geometry_msgs/PoseStamped` | текущая поза кончика по энкодерам |
| `/arm/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | команды моторам 11–17 / 21–27 |

Пример: правая рука в точку перед собой (подставьте свою высоту и ориентацию).

```bash
ros2 topic pub --once /arm_ik/goal_right geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: pelvis}, pose: {position: {x: 0.35, y: -0.25, z: 0.25}, orientation: {w: 1.0}}}"
ros2 topic echo /arm_ik/right/status
```

Если ориентация не важна (или сначала нужно просто проверить досягаемость), запускайте с
`rot_weight:=0.0` — тогда IK решает только по позиции.

## Параметры

| Параметр | По умолчанию | |
|---|---|---|
| `tip_link_left`, `tip_link_right` | `wrist_roll_{l,r}_link` | звено-кончик; можно указать звено ладони из URDF |
| `tcp_offset` | `[0, 0, 0]` | смещение кончика в рамке этого звена, м |
| `rot_weight` | 1.0 | вес ориентации в IK; 0 — только позиция |
| `tol_pos`, `tol_rot` | 0.01 м, 0.05 рад | допуск для `reached` (IK решает с половиной допуска) |
| `kp` | 0.7 | доля ошибки суставов, на которую сдвигается команда при дотягивании |
| `joint_tol` | 0.005 рад | когда дотягивание прекращается |
| `max_corrections` | 15 | предел шагов дотягивания |
| `spd` | 0.8 | максимальная скорость суставов, рад/с |

## Использование как библиотеки

Ядро не зависит от ROS, его можно вызывать прямо из своей ноды:

```python
from arm_ik.kinematics import Arm
arm = Arm(urdf_xml, "right")                     # urdf_xml — строка из /robot_description
ok, q, err_pos, err_rot = arm.solve(T_goal_4x4, q_now, extra=joint_states_dict)
```

## Тесты

```bash
python3 -m pytest -q src/arm_ik/test     # путь к пакету в вашем workspace
```

Тесты используют синтетический URDF с той же структурой суставов; на настоящем URDF робота пакет
проверяется в симуляторе.
