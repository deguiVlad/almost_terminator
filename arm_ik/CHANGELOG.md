# CHANGELOG — arm_ik

## v0.2.2 — bootstrap устойчив к отсутствию numpy/scipy

* `bin/_bootstrap.py` переписан: при нехватке ЛЮБОГО из `rclpy/numpy/scipy` он
  теперь перебирает кандидатные интерпретаторы (текущий, `ARM_IK_PYTHON`,
  `/usr/bin/python3`) и автоматически перезапускается под тем python, где есть
  вся связка; если такой не найден — печатает точную команду установки для
  конкретного интерпретатора (`<python> -m pip install numpy scipy`).
* README §3.3: добавлена строка разбора ошибки `arm_ik: не хватает numpy/scipy`.

## v0.2.1 — запуск без ошибок (исправления по итогам эксплуатации)

Все проблемы, на которые наткнулись при развёртывании, исправлены в пакете и
документированы в README §2–§3:

* **`libexec directory ... does not exist`** — `setup.py` теперь ставит
  standalone-скрипты `bin/*.py` + `_bootstrap.py` в `lib/arm_ik/`, добавлены
  `entry_points`. Лечится полной пересборкой (`rm -rf build/arm_ik install/arm_ik`).
* **`ImportError: cannot import name 'main' ... circular import`** — устранена
  коллизия имён: код нод живёт в `arm_ik/nodes/arm_ik_node.py`, а исполняемые
  обёртки называются иначе (`run_arm_ik.py`, entry_point `arm_ik`). Старые
  артефакты с битой обёрткой нужно удалить (инструкция в README §3).
* **`ModuleNotFoundError: No module named 'rclpy'`** — `bin/_bootstrap.py`
  проверяет наличие rclpy и при необходимости перезапускается под
  `/usr/bin/python3`; отдельного venv для пакета не требуется.
* **Старый оверлей `/nssr_sim` перекрывал свежий пакет** — README §1 объясняет
  порядок `source` и команду `ros2 pkg prefix arm_ik` для диагностики; launch
  отдаёт приоритет скриптам из исходников пакета.
* **pytest 9 падает на стороннем плагине libtmux** — добавлен `pytest.ini`
  (`-p no:libtmux`); тесты ядра: 11 passed.

## v0.2.0 — дельта-управление и документация

* Управление позой кисти двумя способами: абсолютная поза `PoseStamped` на
  `/arm_ik/goal_{left,right}` и шестимерное смещение `TwistStamped` на
  `/arm_ik/delta_{left,right}` (параметры `delta_reference`, `delta_accumulate`,
  предохранители `delta_max_lin/rot`).
* Демо-ноды: `demo_goal_publisher`, `demo_delta_publisher` (+launch-опции
  `demo:=true`, `demo_delta:=true`).
* Подробный README (структура, интерфейсы, конвейер, параметры, troubleshooting).

## v0.1.0 — первая версия

* Ядро без ROS: FK, геометрический Якобиан 6×7, IK Damped Least Squares с
  clip по пределам URDF, multi-seed, null-space; `TableGuard` (клампинг цели,
  барьер в IK, валидация траектории); трапециевидный профиль ≤ 0.8 рад/с.
* ROS-нода: цикл 50 Гц, исполнение через `/arm/cmd_pos` (bodyctrl_msgs,
  моторы 11–17 / 21–27), фаза fine — замыкание контура по `/joint_states`
  (компенсация недолёта ПД-приводов до 15 %), статусы и TCP-поза в топиках.
* `scripts/export_kinematics_from_urdf.py` — генерация `config/arms.json`
  из URDF (авто-поиск файла, TCP через fixed-цепочку от запястья).
