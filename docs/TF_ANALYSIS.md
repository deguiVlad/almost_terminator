# Анализ топика /tf в ROS 2

## 1. Что такое `/tf`?

Топик **`/tf`** в ROS (Robot Operating System) используется для публикации трансформационных данных между различными системами координат в среде робота. Это фундаментальная концепция, необходимая для задач локализации, навигации, отслеживания объектов и управления робототехническими системами.

### Основные сообщения:
- **`tf2_msgs/TFMessage`** - сообщение, содержащее массив трансформаций
- **`geometry_msgs/TransformStamped`** - отдельное сообщение о трансформации одного кадра относительно другого

---

## 2. Как работает `/tf`?

Топик **`/tf`** публикует изменения трансформаций координатных систем в реальном времени. Данные трансформации включают:
- **frame_id** (child frame) - начало системы координат
- **parent_frame_id** (parent frame) - родительская система координат  
- **header.stamp** - временная метка сообщения
- **transform.translation** - позиция начала координат (x, y, z)
- **transform.rotation** - ориентация в quaternion (qx, qy, qz, qw)

---

## 3. TF2: Библиотека трансформаций ROS 2

ROS 2 предоставляет библиотеку **TF2**, которая включает:
- **`tf2_ros/Buffer`** - хранилище преобразований для запроса
- **`tf2_ros/TransformListener`** - слушатель, который ищет трансформации в Buffer
- **`tf2_geometry_msgs`** - утилиты для конвертации TF в geometry_msgs

### Использование в коде:

```cpp
// Создание буфера и слушателя
tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

// Запрос трансформации
try {
    ts = tf_buffer_->lookupTransform(camera_frame_, wrist_frame(),
                                     rclcpp::Clock().now(),
                                     rclcpp::Duration(0, 50'000'000)); // ~50мс истечение
} catch (const tf2::ExtrapolationException &) {
    return; // Трансформация не найдена в будущем
} catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Ошибка TF: %s", ex.what());
}
```

---

## 4. Использованный топик `/tf` в hand_tracker_node

### Описание узла `hand_tracker_node`:

Этот узел **случает за положение кисти робота** через TF2, не используя зрение. Он берет трансформацию от системы координат запястья (`wrist_roll_{l,r}_link`) до системы камеры головы (`camera_head_link`).

### Интерфейсы:
| Топик | Тип сообщения | Направление | Описание |
|-------|--------------|------------|----------|
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | Вход | Трансформации из robot_state_publisher |
| `/cam_head/color/camera_info` | sensor_msgs/CameraInfo | Вход | Интринсики камеры для отладки |
| `/focus/hand` | geometry_msgs/PoseStamped | Выход | Целевая позиция кисти (frame_id=camera_head_link) |
| `/focus/hand_visible` | std_msgs/Bool | Выход | Флаг видимости кисти в поле зрения |

### Математика трансформации:

```cpp
// Запрос трансформации из координат запястья в координаты камеры
ts = tf_buffer_->lookupTransform(camera_frame_, wrist_frame(),
                                 rclcpp::Clock().now());

// Получаем позицию запястья в системе камеры
const auto &t = ts.transform.translation;
double X = t.x, Y = t.y, Z = t.z;  // Оптическая система: x→право, y↓вниз, z→вперёд
```

### Цепочка координатных систем:

Из URDF и `/joint_states` строится следующая цепочка трансформаций:
```
pelvis → torso → head_yaw → head_pitch → camera_head_link 
← wrist_pitch ← wrist_roll_{l,r}_link
```

Дерево TF строится автоматически из URDF (`assets/robots/tiangang_pro/`) и состояния суставов `/joint_states`.

## 5. Запуск узлов с использованием ros.sh

### Правильный способ запусков ROS2:

Чтобы использовать команды ROS2 в симуляторе, необходимо использовать структуру скрипта **`ros.sh`**.

#### Пример использования:

```bash
# Запуск hand_tracker_node для левой руки
bash ~/nssr-polygon/hackathon/tools/ros.sh "source install/setup.bash && \
     ros2 run focus_tracking hand_tracker_node --ros-args -p use_sim_time:=true -p hand:=left"
```

#### Разбор команды:

| Часть | Описание |
|-------|----------|
| `source install/setup.bash` | Загружает окружение ROS2 workspace из установленного пакета |
| `ros2 run focus_tracking hand_tracker_node` | Запускает узел hand_tracker_node из пакета focus_tracking |
| `-p use_sim_time:=true` | Указывает использованию симуляционного времени (из `/clock`) |
| `-p hand:=left` | Выбирает сторону руки: `right` или `left` |

### Полный пример запуска:

```bash
# Запуск с параметрами через скрипт ros.sh
bash ~/nssr-polygon/hackathon/tools/ros.sh "source install/setup.bash && \
     ros2 run focus_tracking hand_tracker_node --ros-args -p use_sim_time:=true -p hand:=left"

# Проверка статуса узла (не в симуляторе)
ros2 node info /hand_tracker_node

# Просмотр топиков в реальном времени
ros2 topic echo /focus/hand
ros2 topic echo /focus/hand_visible

# Просмотр трансформаций TF2
bash ~/nssr-polygon/hackathon/tools/ros.sh "ros2 run tf2_ros tf2_echo pelvis camera_head_link"
```

### Альтернативный способ запуска (без ros.sh):

```bash
# Прямой запуск без скрипта (работает локально)
source ~/nssr-polygon/hackathon/ros2/install/setup.bash && \
ros2 run focus_tracking hand_tracker_node --ros-args -p use_sim_time:=true -p hand:=left
```

---

## 6. Параметры и конфигурация hand_tracker_node

### Параметры узла (через `--ros-args`):

| Параметр | Тип | Значение по умолчанию | Описание |
|----------|-----|----------------------|----------|
| `camera_frame` | string | `"camera_head_link"` | Ссылка камеры в TF |
| `hand` | string | `"right"` | `"left"` или `"right"` |
| `rate_hz` | double | `50.0` | Частота обновления (Гц) |
| `z_near` | double | `0.15` | Минимальная дистанция в мм |

### Пример задания параметров:

```bash
# Запуск правой руки (по умолчанию)
ros2 run focus_tracking hand_tracker_node --ros-args -p use_sim_time:=true -p hand:=right -p rate_hz:=50.0

# Запуск левой руки с другой частотой
ros2 run focus_tracking hand_tracker_node --ros-args \
  -p use_sim_time:=true \
  -p hand:=left \
  -p rate_hz:=100.0
```

---

## 7. Обработка ошибок TF2

### Типы исключений:

| Исключение | Описание | Действие узла |
|------------|----------|---------------|
| `tf2::ExtrapolationException` | Трансформация не найдена (будущая) | Молча пропускает итерацию |
| `tf2::TransformException` | Неверная цепочка координат | Выводит предупреждение и пропускает итерацию |

### Пример обхода ошибок:

```cpp
try {
    ts = tf_buffer_->lookupTransform(camera_frame_, wrist_frame(),
                                     rclcpp::Clock().now());
} catch (const tf2::ExtrapolationException &) {
    RCLCPP_INFO(get_logger(), 
                "Joint_states ещё не пришли — пропускаем итерацию");
    return;  // Пропускает цикл без ошибок
}
```

---

## 8. Отладка и верификация

### Полезные команды:

#### Просмотр топиков:
```bash
# Позиция кисти в системе камеры
ros2 topic echo /focus/hand

# Видимость кисти
ros2 topic echo /focus/hand_visible
```

#### Проверка TF2:
```bash
# Эхо трансформации между кадрами
bash ~/nssr-polygon/hackathon/tools/ros.sh "ros2 run tf2_ros tf2_echo \
     pelvis camera_head_link"

# Просмотр всех преобразований в формате текста
ros2 run tf2_tools tf2_tools view_frames
```

#### Мониторинг узла:
```bash
# Статус узла hand_tracker_node
ros2 node info /hand_tracker_node

# Просмотр логов в реальном времени
tail -f ~/.ros2-logs/hand_tracker_node.log
```

---

## 9. Связанная документация

Для получения дополнительной информации смотрите:

| Файл | Описание |
|------|----------|
| `FOCUS_TRACKING.md` | Математика и архитектура системы фокусировки |
| `INTERFACES.md` | Полный список интерфейсов системы Tiangung |
| `cheatsheet.md` | Справочник по основным топикам и командам |

---

## 10. Частые вопросы (FAQ)

### Q: Что такое `/tf_static`?
**A:** Топик `/tf_static` публикует статические преобразования (не меняющиеся во времени). Некоторые трансформации из URDF публикуются на этом топике, тогда как динамические — на `/tf`.

### Q: Почему нужно использовать `use_sim_time:=true`?
**A:** В симуляторе время управляется через `/clock`, а не через реальное системное время. Без этого параметра узлы будут работать с рассинхронизированными временными метками.

### Q: Как добавить новое соединение к дереву TF?
**A:** Необходимо убедиться, что URDF содержит все требуемые трансформации и публикация из `/joint_states` включает нужные данные о состояниях суставов.

### Q: Почему трансформация не находитсcя (ExtrapolationException)?
**A:** Возможные причины:
- Joint_states ещё не опубликованы
- Ошибка в цепочке координатных систем
- Разный режим времени (симуляция/реальность)

---

## 11. Заключение

Топик **`/tf`** является ключевым компонентом системы Tiangung, обеспечивая точное позиционирование кисти через вычисления TF2. Правильное использование этой технологии требует:
- Правильной конфигурации ROS2 окружения
- Использования скрипта `ros.sh` для запуска узлов в симуляторе
- Понимания структуры координатных систем из URDF

**Важно:** Все ROS2 команды в симуляторе должны выполняться через:
```bash
bash ~/nssr-polygon/hackathon/tools/ros.sh "команда_ROS2"
```

Этот документ был создан на основе анализа исходного кода `hand_tracker_node.cpp` и документации системы Tiangung.

---

*Создано: 11/3/2026 | Анализ hand_tracker_node из focus_tracking/packаgе*
