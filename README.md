# ros2 среда для хакатона ROS Humanoid hack подключенная к репозиторию организаторов

![prev](robot001000.png)

Только разбираемся с репозиторием

## Свои пакеты

- [`focus_tracking/`](focus_tracking) — фокусировка головы на объектах / руках / обоих сразу (HSV+depth детектор, TF2 hand-tracker, контроллер c deadzone/LPF). Математика и архитектура: [docs/FOCUS_TRACKING.md](docs/FOCUS_TRACKING.md). (нестабильно)

- [`arm_ik/`](arm_ik) — Python DLS IK для обеих 7-DOF рук; кинематика и limits загружаются из URDF, публикация команд защищена watchdog обратной связи. Пакет проходит полный аудит; исправления и регрессионные тесты находятся в отдельном draft PR (ROS 2 Humble/hardware validation pending).

- [`arm_ik_kdl/`](arm_ik_kdl) — отдельный C++/KDL сервер IK для ROS 2 Humble: 7-DOF цепи обеих рук из URDF, абсолютные PoseStamped и относительные TransformStamped цели, публикация суставных решений без прямого управления приводами.
