# arm_ik — обратная кинематика обеих рук TienKung 2.0 Pro (ROS 2, Humble)

Пакет решает задачу: **вывести TCP (виртуальный кончик) каждой из двух 7-DOF рук
робота в заданную точку с заданной ориентацией относительно корпуса**, соблюдая
пределы суставов из URDF, лимит скорости моторов ≤ 0.8 рад/с и не задевая стол.
Команды уходят штатными топиками реального робота (`/arm/cmd_pos`), точность
достигается замыканием контура по `/joint_states`.

```
docs (PoseStamped | TwistStamped)            /joint_states
        │                                          │
        ▼                                          ▼
  on_goal / on_delta ──► цель T_des (4×4 в pelvis) ──► IK (DLS, multi-seed,
        барьер стола, clip по limits)  ──► q_sol ──► траектория (trapezoid,
        ≤0.8 рад/с, валидация коллизий) ──► /arm/cmd_pos (50 Гц) ──► приводы
                                        ▲
        фаза fine: dq = J⁺·Kp·e по энкодерам (недолёт ПД до 15 %) ─┘
```

---

## 1. Структура пакета

| Файл | Роль |
|---|---|
| `arm_ik/kinematics.py` | Математическое ядро (без ROS): цепочка JointSpec/ArmModel, FK, геометрический Якобиан 6×7, ошибка позы, IK DLS (Levenberg–Marquardt) с пределами/barrier/null-space, multi-seed, `delta_to_T` для смещений. |
| `arm_ik/collision.py` | `TableGuard`: стол как полуплоскость; клампинг цели, мягкий барьерный градиент внутрь IK, жёсткая проверка точек траектории. |
| `arm_ik/trajectory.py` | `joint_space_profile`: трапециевидный профиль в пространстве суставов с гарантией \|dq/dt\| ≤ v_max. |
| `arm_ik/nodes/arm_ik_node.py` | ROS 2 узел: топики целей (абсолютные и delta), сервис `~/solve`, цикл исполнения 50 Гц, фазы traj→fine→reached, телеметрия. |
| `arm_ik/nodes/demo_goal_publisher.py` | Демо абсолютных целей (PoseStamped). |
| `arm_ik/nodes/demo_delta_publisher.py` | Демо шестимерных смещений (TwistStamped). |
| `bin/arm_ik_node.py`, `bin/demo_goal_publisher.py`, `bin/demo_delta_publisher.py` | Standalone-запускалки нод: исполняемые скрипты с shebang и bootstrap'ом `sys.path` — работают **без colcon build** (`./bin/arm_ik_node.py`). Копируются в `lib/arm_ik/` при сборке. |
| `scripts/export_kinematics_from_urdf.py` | Извлекает точные origin/axis/limits обеих рук из URDF → `config/arms.json`. |
| `config/arms_template.json` | Шаблон модели (топология tiangong_pro + пределы из docs); заменить файлом из URDF! |
| `srv/SolveIK.srv` | Сервис «решить IK без движения» (см. §6 про сборку srv). |
| `test/test_kinematics.py` | 10 офлайн-тестов ядра (FK/Якобиан/IK/траектория/стол/delta). |

## 2. Установка и сборка

```bash
# зависимости: ros2 humble, python3-numpy, python3-scipy (входят в desktop образ)
cd <ws>/src
git clone <этот пакет> arm_ik      # или скопируйте папку
colcon build --packages-select arm_ik
source install/setup.bash
```

**Обязательно пересобирать пакет после любой правки в `setup.py`** (например, при
добавлении исполняемых скриптов) и после добавления новых файлов в `config/` или
`launch/`. Без пересборки в `install/arm_ik/lib/arm_ik/` не появляется директория
`libexec`, и `ros2 launch` падает с ошибкой вида:

```
package 'arm_ik' found at '.../install/arm_ik', but libexec directory
'.../install/arm_ik/lib/arm_ik' does not exist
```

Причины этой ошибки:
1. пакет собран до появления `entry_points`/`data_files` — сделать `colcon build --packages-select arm_ik`;
2. сборка без symlink и старый `install/` — надёжнее `colcon build --packages-select arm_ik --symlink-install`;
3. в другом терминале активирован старый оверлей (`source .../install/setup.bash` из другой копии workspace) — перезейтить `source install/setup.bash` в текущем терминале;
4. проверьте наличие ноды: `ros2 pkg executables arm_ik` — должны быть `arm_ik_node`, `demo_goal_publisher`, `demo_delta_publisher`, `export_kinematics_from_urdf`.


### 2.5. Если ошибка `ImportError: cannot import name 'main' ... circular import`

Причина (исправлена в текущей версии пакета): скрипт-обёртка `lib/arm_ik/arm_ik_node.py`
импортировал модуль с тем же именем `arm_ik.arm_ik_node`; Python подменял имя модуля
скриптом (`__main__`) и получал самозамкнутый импорт. Теперь код ноды живёт в
`arm_ik/nodes/arm_ik_node.py`, а обёртки в `bin/` импортируют `arm_ik.nodes.arm_ik_node`.

Что сделать на вашей машине (обязательно с чистой пересборкой):
```bash
cd ~/nssr-polygon            # корень workspace с пакетом arm_ik
# 1) синхронизировать исправленный пакет (git pull / копирование arm_ik/)
# 2) вычистить СТАРЫЕ артефакты сборки — именно они содержат битую обёртку:
rm -rf build/arm_ik install/arm_ik log/build_arm_ik* log/latest_build
colcon build --packages-select arm_ik --symlink-install
source install/setup.bash
# 3) проверка, что импорт резолвится:
ros2 pkg executables arm_ik
python3 -c "from arm_ik.nodes.arm_ik_node import main; print('import ok')"
ros2 launch arm_ik arm_ik.launch.py
```
Если после этого снова виден путь `/nssr_sim/install/arm_ik/lib/arm_ik/arm_ik_node.py`
с прежним содержимым — значит source'ится другое окружение (`/nssr_sim/install/setup.bash`),
в нём пакет старый: пересоберите arm_ik именно в том workspace, чей setup.bash вы используете.

Запуск вместе с симулятором требует того же окружения, что и остальные пакеты
команды (`docs/cheatsheet.md`): `export ROS_DOMAIN_ID=146`, `use_sim_time=true`.

### 2.1 Быстрый запуск без colcon build (если сборка упорно не проходит)

Ноды можно запускать напрямую — скрипты в `bin/` сами добавляют путь к пакету
и при необходимости перезапускаются под интерпретатором, в котором есть `rclpy`
(`/usr/bin/python3`, системные пакеты ROS):

```bash
source /opt/ros/humble/setup.bash          # только ROS, workspace не нужен
cd <путь>/arm_ik
python3 ./bin/arm_ik_node.py --ros-args -p use_sim_time:=true -p h_table:=0.9 &
python3 ./bin/demo_goal_publisher.py --ros-args -p use_sim_time:=true   # опционально
```

Если появляется `ModuleNotFoundError: No module named 'rclpy'` — значит нода
стартует от «чужого» python (venv/conda). Быстрая проверка и лечение:

```bash
which python3                              # должен быть /usr/bin/python3
/usr/bin/python3 -c "import rclpy; print('rclpy ok')"
# если в venv: deactivate; если conda: conda deactivate
```

После нормальной сборки (`colcon build`) preferred-way — без расширения:
```bash
ros2 run arm_ik arm_ik_node                # entry_point console_scripts
# или standalone-бинарь, ставящийся в lib/arm_ik/:
install/<ws>/lib/arm_ik/arm_ik --ros-args -p use_sim_time:=true
```

launch-файл тоже умеет работать из исходников: `_node_exec()` сначала ищет
исполняемый в собранном `lib/arm_ik/`, а при его отсутствии берёт `bin/*.py`
рядом с исходным пакетом. Но сам `ros2 launch arm_ik ...` требует, чтобы пакет
хотя бы раз был собран (ament должен знать `arm_ik`) — для совсем «без сборки»
запускайте ноды напрямую из `bin/`, как выше.

Полезная диагностика:
```bash
ros2 pkg executables arm_ik                 # список установленных нод
ls install/arm_ik/lib/arm_ik/               # должны лежать *.py из bin/ и scripts/
python3 -c "from ament_index_python.packages import get_package_prefix; print(get_package_prefix('arm_ik'))"
```

## 3. Подготовка модели руки (обязательно перед боем)

Шаблонные длины звеньев в `arms_template.json` приблизительны. Снимите точные
параметры из URDF робота:

```bash
# путь к URDF можно не указывать — скрипт сам найдёт tiangong_pro_with_hands.urdf
# (ищет в ~/nssr-polygon, ~/, /nssr_sim, /workspace); либо явно:
python3 scripts/export_kinematics_from_urdf.py \
    --urdf ~/nssr-polygon/assets/robots/tiangong_pro/tiangong_pro_with_hands.urdf \
    --out config/arms.json

# TCP = кончик кисти: tip_link берётся из URDF автоматически (обход fixed-цепочки
# от wrist_roll до листа, обычно hand_{l,r}_link); сместить его можно так:
#   --tcp-offset 0.1,0,0        # или перебить имя звена: --tip-link-left hand_l_link

# если файл недоступен на диске — возьмите модель с живой системы:
ros2 topic echo --once /robot_description data | sed -n '2,$p' > /tmp/robot.urdf
python3 scripts/export_kinematics_from_urdf.py --urdf /tmp/robot.urdf --out config/arms.json
```

Если имена суставов в URDF отличаются от канонических (`shoulder_pitch_l_joint` и т.д.
из `docs/INTERFACES.md`), скрипт подберёт их по ключевым словам; при неоднозначности
он напечатает список всех суставов — допишите реальное имя в `KEYWORDS`/`MOTOR_IDS`.

Узел автоматически подхватит `config/arms.json` (или путь из параметра
`model_path`; launch ищет его в share-папке пакета).

Высота стола задаётся параметром `h_table` (м, над `pelvis`) и прямоугольником
`table_x0/x1/y_half` — измерьте один раз по TF/линейке или по позам объектов.

## 4. Интерфейсы узла

### 4.1 Абсолютная поза: `geometry_msgs/PoseStamped` на `/arm_ik/goal_left|goal_right`

Шестимерное положение TCP кисти. `header.frame_id` — любая рамка дерева TF
(узел перенесёт в `pelvis` через TF2); пустая/`pelvis` — уже в корпусе.
Ориентация — кватернион `pose.orientation`.

```bash
ros2 topic pub -1 /arm_ik/goal_right geometry_msgs/msg/PoseStamped "{
  header: {frame_id: pelvis},
  pose: {position: {x: 0.45, y: -0.25, z: 1.0},
         orientation: {x: 0.7071, y: 0.0, z: 0.0, w: 0.7071}}}"
```

### 4.2 Смещение: `geometry_msgs/TwistStamped` на `/arm_ik/delta_left|delta_right`

Шестимерное **относительное** смещение текущей позы TCP:
* `twist.linear.{x,y,z}` — dxyz в метрах, ось = `base_frame` (pelvis: x вперёд, y влево, z вверх);
* `twist.angular.{x,y,z}` — вращение как rotation vector (ось × угол, радианы), тоже в pelvis.

Цель вычисляется как `T_new = Δ · T_ref`:
`p_new = p_ref + dpos`, `R_new = Rot(drot) · R_ref`.

Что такое `T_ref` — выбирается параметром **`delta_reference`**:

| значение | смысл | поведение при повторе той же дельты |
|---|---|---|
| `measured` (умолч.) | текущая **измеренная** FK-поза TCP по `/joint_states` | «шаговый» режим: каждая публикация двигает ещё на d от фактического положения (удобно для джойстика/телеоперации) |
| `last_goal` | последняя поставленная цель (абсолютная или delta) | идемпотентно: рука встанет в одну и ту же точку |

Дополнительно:
* **`delta_accumulate:=true`** — series-режим: дельты суммируются от последней
  delta-цели серии (каждая новая команда — продолжение движения), пока не
  пришла абсолютная цель или рука не в `idle/failed`;
* **`delta_max_lin` (0.15 м) / `delta_max_rot` (0.6 рад)** — предохранители:
  слишком большая одна дельта отклоняется с предупреждением (защита от опечаток
  и залипания джойстика; серию больших перемещений делайте маленькими шагами);
* нулевая дельта игнорируется; чужая `frame_id` в delta не переносится через TF
  (трактуется как base_frame, пишется warning) — это осознанно: delta имеет
  смысл только в системе корпуса.

После вычисления `T_des` delta-цель проходит **тот же конвейер**, что и
абсолютная: клампинг выше стола → IK → валидация траектории → исполнение →
fine-дотягивание. То есть «поднять кисть на 3 см над столом» безопасно даже
если текущая поза была у самой поверхности.

```bash
# сдвинуть правую кисть на 5 см вперёд и довернуть на 0.2 рад вокруг вертикали
ros2 topic pub -1 /arm_ik/delta_right geometry_msgs/msg/TwistStamped "{
  header: {frame_id: pelvis},
  twist: {linear: {x: 0.05}, angular: {z: 0.2}}}"

# демо-«шагоход»: каждые 6 с +5 см вперёд
ros2 run arm_ik demo_delta_publisher --ros-args -p use_sim_time:=true \
    -p side:=right -p dx:=0.05 -p period:=6.0
```

### 4.3 Выходы

| Топик | Тип | Содержимое |
|---|---|---|
| `/arm/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | пачка `SetMotorPosition(name=motor_id, pos, spd≤0.8, cur)` для ID 11..17 / 21..27, 50 Гц |
| `/arm_ik/wrist_{left,right}_pose` | `PoseStamped` (pelvis) | текущая FK-поза TCP по энкодерам — видно в RViz, полезно верхнему уровню |
| `/arm_ik/{left,right}/status` | `std_msgs/String` (JSON) | `state: idle|traj|fine|reached|failed`, `err_pos_m`, `err_rot_rad` |
| `~/solve` | `arm_ik/srv/SolveIK` | решить IK без движения (для планировщика верхнего уровня) |

## 5. Как это работает внутри (по этапам)

1. **Постановка цели.** PoseStamped переносится TF2 в `pelvis`; TwistStamped —
   прибавляется к опорной позе (§4.2). Результат — матрица 4×4 `T_des`.
2. **Клампинг стола.** Если TCP-точка цели лежит в прямоугольнике стола ниже
   `h_table + margin`, она поднимается ровно на поверхность+запас
   (`TableGuard.clamp_target_above_table`).
3. **IK (Damped Least Squares).** Итеративно: `dq = W Jᵀ (J W Jᵀ + λ²I)⁻¹ e`,
   где `e` — [Δp; rotvec(R_err)], `J` — геометрический якобиан 6×7 TCP. Каждый
   шаг: плюс барьерный градиент от стола, минус проекция в нуль-пространстве к
   предпочтительной позе (использует 7-ю степень свободы, позу не меняет), затем
   **clip по пределам URDF**. Стартов несколько (текущая измеренная + типовые
   позы) — перебор лечит локальные минимумы (`ik_multi_seed`).
4. **Траектория.** Трапециевидный профиль q_now→q_sol, синхронизированный по
   самой долгой оси, с численной проверкой `\|Δq\|/dt ≤ 0.8` (страховочное
   растяжение при квантовании). Каждая точка профиля проверяется
   `collides_with_table`; если прямая траектория задевает стол — автоматический
   **план Б**: waypoint строго над столом (+`waypoint_lift`), две фазы профиля
   («заход сверху», приём FOCUS_TRACKING.md).
5. **Исполнение.** На 50 Гц публикуется очередная точка профиля в `/arm/cmd_pos`.
6. **Фаза fine (замкнутый контур).** Приводы — ПД с недолётом до 15 %, поэтому
   после профиля ошибка позы добивается по энкодерам:
   `dq = Jᵀ(JJᵀ+λ²I)⁻¹ e`, клип ±0.1 рад/цикл (рекомендация docs), Kp≈0.6,
   пока `err_pos < tol_pos (1 см)` и `err_rot < tol_rot (0.05 рад)` → `reached`.
   Страховка `fine_max_cycles` не даёт зациклиться.
7. **Отказ.** Если IK не сошлась (цель вне workspace/в пределах невозможно) —
   состояние `failed`, рука удерживается, в статусе публикуется остаточная
   ошибка; узел готов принять следующую цель.

Обе руки — независимые стан-машины в одном узле: общий топик команд, непересекающиеся ID моторов.

## 6. Про сервис `SolveIK`

В чистом `ament_python` rosidl-кодген srv не запускается автоматически. Варианты:
* не использовать сервис (узел полностью работает от топиков — код защищён
  `try/except ImportError`);
* собрать srv в соседнем `ament_cmake` пакете-обёртке и добавить
  `exec_depend` (пример в комментариях `setup.py` — раскомментировать glob и
  `generate_messages`);
* временно заменить сервис на пару стандартных message-топиков.

## 7. Параметры узла (кратко)

| Параметр | Умолч. | Назначение |
|---|---|---|
| `rate_hz` | 50 | частота цикла и дискретизация профиля |
| `spd_limit` | 0.8 | жёсткий потолок скорости (min с документным лимитом) |
| `a_max` | 0.4 | ускорение профиля, рад/с² |
| `tol_pos`/`tol_rot` | 0.01/0.05 | критерии достижения, м/рад |
| `kp`, `fine_max_cycles` | 0.6, 300 | коррекция недолёта |
| `h_table`, `table_x0/x1/y_half/margin` | 0.9, 0.35/1.2/1.0/0.02 | геометрия стола |
| `enable_table_guard` | true | выключить защиту (например, для задач вне стола) |
| `waypoint_lift` | 0.25 | запас захода сверху, м |
| `n_seeds` | 4 | число стартов IK |
| `delta_reference` | measured | опора delta-целей (§4.2) |
| `delta_accumulate` | false | суммирование серии дельт |
| `delta_max_lin/rot` | 0.15/0.6 | предохранители размера дельты |
| `tcp_offset` | "" | переопределить "x,y,z" кончика поверх arms.json |
| `model_path` | "" | arms.json (пусто — из share/config) |

## 8. Быстрый старт

`arm_ik_node` — **постоянная нода-сервер**: запускается один раз и крутится
бесконечно (`rclpy.spin`, цикл 50 Гц), ожидая цели на топиках; она не завершается
после достижения цели и принимает новые цели в любой момент. Останавливается
только вручную (Ctrl+C) или при падении `/joint_states`.

```bash
# терминал 1 — симулятор/реальный робот по cheatsheet команды...

# терминал 2 — запустить IK-сервер как постоянную ноду (без демонстрационных целей):
source install/setup.bash
ros2 launch arm_ik arm_ik.launch.py                 # — рекомендуется через launch
#   или напрямую:
ros2 run arm_ik arm_ik_node --ros-args -p use_sim_time:=true -p h_table:=0.9
#   (в фоне: ... > /tmp/arm_ik.log 2>&1 &)

# теперь цели можно слать когда угодно, сервер их подхватывает:
ros2 topic pub -1 /arm_ik/goal_right geometry_msgs/msg/PoseStamped \
  "{header:{frame_id:pelvis}, pose:{position:{x:0.45,y:-0.2,z:1.0}, orientation:{w:1}}}"
ros2 topic pub -1 /arm_ik/delta_right geometry_msgs/msg/TwistStamped \
  "{header:{frame_id:pelvis}, twist:{linear:{x:0.05}}}"

# вариант А — абсолютные демо-цели одним запуском:
ros2 launch arm_ik arm_ik.launch.py demo:=true h_table:=0.9 goal_x:=0.45 goal_z:=1.0
# вариант Б — delta-«шагоход» (+5 см вперёд каждые 6 с, правая рука):
ros2 launch arm_ik arm_ik.launch.py demo_delta:=true delta_side:=right delta_x:=0.05
# терминал 3 — delta-телеоперация правой руки:
ros2 topic pub -1 /arm_ik/delta_right geometry_msgs/msg/TwistStamped \
  "{header:{frame_id:pelvis}, twist:{linear:{x:0.05, z:-0.03}}}"
# мониторинг:
ros2 topic echo /arm_ik/right/status
ros2 topic echo /arm_ik/wrist_right_pose
```

## 9. Тесты

```bash
cd arm_ik && python3 -m pytest test -q     # 11 тестов, ROS не нужен
```

Покрывают: детерминированность FK и пределы из docs; совпадение аналитического
Якобиана с конечно-разностным; round-trip IK(FK(q)); скорость профиля ≤ 0.8;
клампинг/детекцию стола; чтение arms.json; арифметику delta-целей и достижимость
delta-цели через IK.

## 10. Известные ограничения

* Модель — 7-звенная цепочка без коллизионной геометрии корпуса/второй руки: стол
  аналитический (плоскость), тело не проверяется (при заходе сверху и малых
  дельтах не требуется; для сложных сцен — MoveIt-вариант).
* DLS — численный метод: экстремальные цели вне workspace дадут `failed`
  (это корректное поведение, не падение).
* Перед использованием обязательна перегенерация `arms.json` из фактического
  URDF и замер `h_table`.
