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

Версии и список изменений — в [CHANGELOG.md](CHANGELOG.md).

---

## 0. Быстрый старт (правильный порядок команд)

```bash
# 0) окружение ROS + домен хакатона (или используйте hackathon/tools/ros.sh)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=146

# 1) workspace, в котором лежит пакет arm_ik (у нас это ~/nssr-polygon/hackathon/ros2)
cd ~/nssr-polygon/hackathon/ros2

# 2) чистая пересборка пакета (обязательно после обновления исходников!)
rm -rf build/arm_ik install/arm_ik log/build_arm_ik*
colcon build --packages-select arm_ik --symlink-install
source install/setup.bash          # source ПОСЛЕДНИМ, чтобы он имел приоритет

# 3) проверка установки
ros2 pkg prefix arm_ik             # должен показать .../hackathon/ros2/install/arm_ik
ros2 pkg executables arm_ik        # arm_ik, demo_goal, demo_delta (+ run_*.py в lib/)

# 4) подготовка модели руки из URDF (один раз; см. §4)
python3 arm_ik/scripts/export_kinematics_from_urdf.py --out arm_ik/config/arms.json

# 5) запуск IK-сервера (постоянная нода, ждёт цели на топиках)
ros2 launch arm_ik arm_ik.launch.py

# 6) в другом терминале — цель (абсолютная поза или смещение, см. §5)
ros2 topic pub -1 /arm_ik/goal_right geometry_msgs/msg/PoseStamped \
  "{header:{frame_id:pelvis}, pose:{position:{x:0.45,y:-0.2,z:1.0}, orientation:{w:1}}}"
```

Если нет времени на сборку — запуск вообще без colcon (§3, вариант «из исходников»).

## 1. Окружение и типичные грабли с путями

Контейнер симулятора сам использует оверлей `/nssr_sim/install/setup.bash`, в
котором живёт **старая** копия `arm_ik`. Python и ROS берут пакет из того
оверлея, который отsource'нут последним / имеет приоритет в `AMENT_PREFIX_PATH`.

Правила:

* `source` делайте в порядке: сначала `/nssr_sim/install/setup.bash` (нужен для
  `bodyctrl_msgs`), **последним** — свой workspace: `~/nssr-polygon/hackathon/ros2/install/setup.bash`;
* проверяйте конфликт одной командой:
  ```bash
  ros2 pkg prefix arm_ik      # где ROS видит пакет
  python3 -c "import arm_ik; print(arm_ik.__file__)"   # какой модуль импортируется
  ```
  оба пути должны вести в ваш свежий workspace (для второй команды — из его корня);
* если в логах запуска появляется путь `/nssr_sim/install/arm_ik/...` — вы
  source'ите старый оверлей: пересоберите пакет в нём
  (`cd /nssr_sim && rm -rf build/arm_ik install/arm_ik && colcon build --packages-select arm_ik`)
  или поменяйте порядок `source`;
* venv/conda ломает доступ к `rclpy` (он только в системном `/usr/bin/python3`):
  `deactivate` перед запуском. Сам bootstrap при отсутствии rclpy умеет
  перезапускаться под `/usr/bin/python3` автоматически.

## 2. Структура пакета

| Файл | Роль |
|---|---|
| `arm_ik/kinematics.py` | Математическое ядро (без ROS): цепочка JointSpec/ArmModel, FK, геометрический Якобиан 6×7, ошибка позы, IK DLS (Levenberg–Marquardt) с пределами/barrier/null-space, multi-seed, `delta_to_T` для смещений. |
| `arm_ik/collision.py` | `TableGuard`: стол как полуплоскость; клампинг цели, мягкий барьерный градиент внутрь IK, жёсткая проверка точек траектории. |
| `arm_ik/trajectory.py` | `joint_space_profile`: трапециевидный профиль в пространстве суставов с гарантией \|dq/dt\| ≤ v_max. |
| `arm_ik/nodes/arm_ik_node.py` | ROS 2 узел (код ноды!): топики целей (абсолютные и delta), сервис `~/solve`, цикл исполнения 50 Гц, фазы traj→fine→reached, телеметрия. |
| `arm_ik/nodes/demo_goal_publisher.py` | Демо абсолютных целей (PoseStamped). |
| `arm_ik/nodes/demo_delta_publisher.py` | Демо шестимерных смещений (TwistStamped). |
| `bin/run_arm_ik.py`, `bin/run_demo_goal.py`, `bin/run_demo_delta.py` | Standalone-запускалки. Имена намеренно НЕ совпадают с именами модулей — это исключает circular import. Работают из исходников и из `install/.../lib/arm_ik/`. |
| `bin/_bootstrap.py` | Единый bootstrap: поиск пакета-исходников, чистка sys.path от битых старых сборок, перезапуск под python с rclpy, человекочитаемые ошибки вместо трейсбеков. |
| `scripts/export_kinematics_from_urdf.py` | Извлекает точные origin/axis/limits обеих рук из URDF → `config/arms.json`. |
| `config/arms_template.json` | Шаблон модели (топология tiangong_pro + пределы из docs); заменить файлом из URDF! |
| `launch/arm_ik.launch.py` | Запуск сервера (+опционально демо); сам находит исполняемый (исходники → entry_point → libexec). |
| `srv/SolveIK.srv` | Сервис «решить IK без движения» (см. §7 про сборку srv). |
| `test/test_kinematics.py` | 11 офлайн-тестов ядра (FK/Якобиан/IK/траектория/стол/delta). |
| `pytest.ini` | Отключает сторонний плагин libtmux (ломает pytest≥9). |

## 3. Установка, сборка, запуск

### 3.1 Нормальный путь (colcon)

```bash
cd <ws>                       # workspace, где лежит папка arm_ik (как пакет, не в src/ тоже ок)
rm -rf build/arm_ik install/arm_ik log/build_arm_ik*
colcon build --packages-select arm_ik --symlink-install
source install/setup.bash
ros2 pkg executables arm_ik   # arm_ik / demo_goal / demo_delta
```

Запуск (нода — постоянный сервер, см. §8):

```bash
ros2 launch arm_ik arm_ik.launch.py
ros2 run arm_ik arm_ik --ros-args -p use_sim_time:=true -p h_table:=0.9   # эквивалент
```

### 3.2 Без сборки — напрямую из исходников пакета

```bash
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=146
cd <путь>/arm_ik
chmod +x bin/*.py
./bin/run_arm_ik.py --ros-args -p use_sim_time:=true -p h_table:=0.9 &
```

`_bootstrap.py` сам подключит пакет к `sys.path`, найдёт python с `rclpy` и
даст понятную ошибку, если окружение не ready.

### 3.3 Разбор ошибок запуска (все три уже исправлены в пакете)

| Симптом | Причина | Лечение |
|---|---|---|
| `libexec directory '.../lib/arm_ik' does not exist` | пакет собран до появления скриптов/entry_points, либо старый `install/` | `rm -rf build/arm_ik install/arm_ik && colcon build --packages-select arm_ik --symlink-install && source install/setup.bash` |
| `ImportError: cannot import name 'main' from partially initialized module 'arm_ik.arm_ik_node' (circular import)` | старая обёртка `lib/arm_ik/arm_ik_node.py` имела то же имя, что модуль, и подменяла его в `sys.modules`. В текущей версии имена разведены (`run_arm_ik.py` ↔ `arm_ik/nodes/arm_ik_node.py`) | удалить СТАРЫЕ артефакты во всех workspace, где собирался пакет: `rm -rf /nssr_sim/{build,install}/arm_ik <ваш_ws>/{build,install}/arm_ik`, затем пересобрать и перезейтить; проверить: `head -8 $(which ...)/install/arm_ik/lib/arm_ik/run_arm_ik.py` и `ros2 pkg prefix arm_ik` |
| `ModuleNotFoundError: No module named 'rclpy'` | запуск из venv/conda или python без системных пакетов ROS | `deactivate`; `source /opt/ros/humble/setup.bash`; либо запускать `./bin/run_arm_ik.py` (bootstrap перезапустится под `/usr/bin/python3`). Проверка: `/usr/bin/python3 -c "import rclpy; print('ok')"` |
| Собрано в одном ws, а launch берёт пакет из другого (`/nssr_sim`) | приоритет оверлеев в `AMENT_PREFIX_PATH` | source'ть свой `install/setup.bash` последним; контроль — `ros2 pkg prefix arm_ik` |
| `python3 -c "from arm_ik.nodes..."` → ModuleNotFoundError | команда запущена вне workspace без `source install/setup.bash` | это нормально: после `source install/setup.bash` импорт работает; standalone-скрипты собирают путь сами |

Диагностика одним блоком:

```bash
ros2 pkg prefix arm_ik
ls install/arm_ik/lib/arm_ik/
grep -n "import" install/arm_ik/lib/arm_ik/run_arm_ik.py
python3 -c "import arm_ik, arm_ik.nodes.arm_ik_node as m; print(m.__file__)"
```

## 4. Подготовка модели руки (обязательно перед боем)

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
`model_path`; launch ищет его в share-папке пакета). При правке `config/`
пересоберите пакет (`--symlink-install` подхватывает изменения json на лету).

Высота стола задаётся параметром `h_table` (м, над `pelvis`) и прямоугольником
`table_x0/x1/y_half` — измерьте один раз по TF/линейке или по позам объектов.

## 5. Интерфейсы узла

### 5.1 Абсолютная поза: `geometry_msgs/PoseStamped` на `/arm_ik/goal_left|goal_right`

Шестимерное положение TCP кисти. `header.frame_id` — любая рамка дерева TF
(узел перенесёт в `pelvis` через TF2); пустая/`pelvis` — уже в корпусе.
Ориентация — кватернион `pose.orientation`.

```bash
ros2 topic pub -1 /arm_ik/goal_right geometry_msgs/msg/PoseStamped "{
  header: {frame_id: pelvis},
  pose: {position: {x: 0.45, y: -0.25, z: 1.0},
         orientation: {x: 0.7071, y: 0.0, z: 0.0, w: 0.7071}}}"
```

### 5.2 Смещение: `geometry_msgs/TwistStamped` на `/arm_ik/delta_left|delta_right`

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
ros2 run arm_ik demo_delta --ros-args -p use_sim_time:=true \
    -p side:=right -p dx:=0.05 -p period:=6.0
```

### 5.3 Выходы

| Топик | Тип | Содержимое |
|---|---|---|
| `/arm/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | пачка `SetMotorPosition(name=motor_id, pos, spd≤0.8, cur)` для ID 11..17 / 21..27, 50 Гц |
| `/arm_ik/wrist_{left,right}_pose` | `PoseStamped` (pelvis) | текущая FK-поза TCP по энкодерам — видно в RViz, полезно верхнему уровню |
| `/arm_ik/{left,right}/status` | `std_msgs/String` (JSON) | `state: idle|traj|fine|reached|failed`, `err_pos_m`, `err_rot_rad` |
| `~/solve` | `arm_ik/srv/SolveIK` | решить IK без движения (для планировщика верхнего уровня) |

## 6. Как это работает внутри (по этапам)

1. **Постановка цели.** PoseStamped переносится TF2 в `pelvis`; TwistStamped —
   прибавляется к опорной позе (§5.2). Результат — матрица 4×4 `T_des`.
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

## 7. Про сервис `SolveIK`

В чистом `ament_python` rosidl-кодген srv не запускается автоматически. Варианты:
* не использовать сервис (узел полностью работает от топиков — код защищён
  `try/except ImportError`);
* собрать srv в соседнем `ament_cmake` пакете-обёртке и добавить
  `exec_depend` (пример в комментариях `setup.py` — раскомментировать glob и
  `generate_messages`);
* временно заменить сервис на пару стандартных message-топиков.

## 8. Параметры узла (кратко)

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
| `delta_reference` | measured | опора delta-целей (§5.2) |
| `delta_accumulate` | false | суммирование серии дельт |
| `delta_max_lin/rot` | 0.15/0.6 | предохранители размера дельты |
| `tcp_offset` | "" | переопределить "x,y,z" кончика поверх arms.json |
| `model_path` | "" | arms.json (пусто — из share/config) |

## 9. Постоянный режим работы

`arm_ik_node` — **постоянная нода-сервер**: запускается один раз и крутится
бесконечно (`rclpy.spin`, цикл 50 Гц), ожидая цели на топиках; она не завершается
после достижения цели и принимает новые цели в любой момент. Останавливается
только вручную (Ctrl+C) или при падении `/joint_states`.

```bash
# терминал 1 — симулятор/реальный робот по cheatsheet команды...

# терминал 2 — IK-сервер:
source install/setup.bash
ros2 launch arm_ik arm_ik.launch.py
#   или напрямую:
ros2 run arm_ik arm_ik --ros-args -p use_sim_time:=true -p h_table:=0.9
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
# мониторинг:
ros2 topic echo /arm_ik/right/status
ros2 topic echo /arm_ik/wrist_right_pose
```

## 10. Тесты

```bash
cd arm_ik && python3 -m pytest test -q     # 11 тестов, ROS не нужен
```

(`pytest.ini` отключает конфликтующий плагин libtmux; если ваш pytest всё же
падает с «Marks cannot be applied to fixtures» — добавьте `-p no:libtmux`.)

Покрывают: детерминированность FK и пределы из docs; совпадение аналитического
Якобиана с конечно-разностным; round-trip IK(FK(q)); скорость профиля ≤ 0.8;
клампинг/детекцию стола; чтение arms.json; арифметику delta-целей и достижимость
delta-цели через IK.

## 11. Известные ограничения

* Модель — 7-звенная цепочка без коллизионной геометрии корпуса/второй руки: стол
  аналитический (плоскость), тело не проверяется (при заходе сверху и малых
  дельтах не требуется; для сложных сцен — MoveIt-вариант).
* DLS — численный метод: экстремальные цели вне workspace дадут `failed`
  (это корректное поведение, не падение).
* Перед использованием обязательна перегенерация `arms.json` из фактического
  URDF и замер `h_table`.
