# Нода фокуса головы (`head_focus_node`)

Пакет: `task_2_cpp` · Симулятор: nssr-sim, ROS 2 Humble.

## Что делает

Подписывает голову на цель (объект в кадре): получает 3D-позицию цели в системе
координат камеры, пересчитывает её в `head_pitch`/`head_yaw` и публикует
позиционные команды в `/head/cmd_pos` (моторы 2 = pitch, 3 = yaw) с частотой 20 Гц.

## Топики / сервисы

| Направление | Интерфейс | Тип | Назначение |
|---|---|---|---|
| В | `/head_focus/target` | `geometry_msgs/PoseStamped` | позиция цели, `frame_id=camera_head_link` |
| В | `/cam_head/color/camera_info` | `sensor_msgs/CameraInfo` | K: fx, fy, cx, cy (для deadzone в пикселях) |
| В | `/head/status` | `bodyctrl_msgs/MotorStatusMsg` | текущие pos моторов (feedback) |
| Вы | `/head/cmd_pos` | `bodyctrl_msgs/CmdSetMotorPosition` | команды `{name, pos[рад], spd[rpm], cur[A]}` |
| Сервис | `/head_focus/set_mode` | `std_srvs/SetBool` | true = ACTIVE, false = IDLE (нейтраль) |

## Математика перевода цели в углы

Камера головы — оптическая система координат (`z` вперёд, `x` вправо, `y` вниз):

```
yaw   = atan2(x, z)          // поворот влево/вправо
pitch = atan2(-y, z)         // кивок вверх/вниз
```

Если детектор отдаёт только 2D-центр bbox `(u, v)` без глубины, эквивалентная
pinhole-формула по внутренним параметрам: `yaw = atan2(u-cx, fx)`,
`pitch = atan2(v-cy, fy)` (глубина не нужна). Промежуточный вариант — взять
значение depth-картины в `(u, v)` и back-projection: `x=(u-cx)·d/fx, y=(v-cy)·d/fy, z=d`.

## Стабилизация и «человечность» движения

1. **Deadzone** (`deadzone_px`, по умолчанию 20 px): если ошибка мала — голова стоит,
   иначе дёрганье на шуме детектора.
2. **LPF-сглаживание** (`smoothing`, α=0.5): `cmd = α·цель + (1−α)·предыдущая`.
3. **Ограничения суставов**: `yaw_limit` 0.7 рад, `pitch_limit` 0.5 рад (clamp).
4. **Таймаут цели** (`target_timeout_s` 1.0): цель пропала → возврат в нейтраль.
5. Командуем pitch и yaw **одним сообщением** — оба мотора идут одновременно.

## Сборка и запуск

```bash
colcon build --packages-select task_2_cpp && source install/setup.bash
ros2 run task_2_cpp head_focus_node --ros-args \
  -p yaw_limit:=0.6 -p pitch_limit:=0.4 -p deadzone_px:=20

# тест без детектора — подать фиктивную цель:
ros2 topic pub -r 10 /head_focus/target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: camera_head_link}, pose: {position: {x: 0.3, y: 0.0, z: 1.5}}}"

# выключить слежение:
ros2 service call /head_focus/set_mode std_srvs/srv/SetBool "{data: false}"
```

## Как подключить детектор объектов

Нужен промежуточный узел `object_detector`, который:
1. подписан на `/cam_head/color/image_raw` (+ `depth/image_raw`);
2. определяет объект (AprilTag/ArUco через `vision_opencv`, цветовой порог, YOLO и т.п.);
3. берёт центр bbox `(u,v)`, читает depth `d` в этой точке;
4. публикует `PoseStamped` в `camera_head_link` на `/head_focus/target`.

Для нескольких объектов — выберите один по стратегии (ближайший / заданный ID),
либо добавьте переключение цели сервисом `/head_focus/select_target`.

## Идеи развития

- **Пан-тильт конечный автомат**: сначала coarse yaw (быстро), затем точный pitch —
  имитация «осознанного взгляда».
- **Упреждение (lead)**: оценка скорости цели по дифференцированию последних N целей,
  команда в упреждённую точку (параметр `lead_gain` уже заложен).
- **Search pattern**: при потере цели — «круги» головой (можно переиспользовать
  траекторию из `head_demo.cpp`), пока детектор снова не найдёт объект.
- **Фокус по размеру bbox**: приблизиться нельзя головой, но можно масштабировать
  deadzone от размера объекта (мелкий объект → tighter deadzone).
- Проверка видимости: перед захватом убедиться, что цель в центре кадра дольше N сек.
