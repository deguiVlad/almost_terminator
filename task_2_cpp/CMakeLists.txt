cmake_minimum_required(VERSION 3.8)
project(task_2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Humble/Iron/Jazzy обычно используют C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Находим ament_cmake
find_package(ament_cmake REQUIRED)

# Находим зависимости ROS 2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(bodyctrl_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Создаём исполняемый файл
add_executable(zahvat_node src/zahvat.cpp)

# Паблишер демонстрации движения головой
add_executable(head_demo_node src/head_demo.cpp)

# Нода поворота головы для фокуса на объектах (visual focus / object tracking)
add_executable(head_focus_node src/head_focus_node.cpp)

# ВАЖНО: именно эта строка передаёт include/link флаги rclcpp и msgs в zahvat_node
ament_target_dependencies(zahvat_node
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
)

ament_target_dependencies(head_demo_node
  rclcpp
  bodyctrl_msgs
  rosgraph_msgs
)

ament_target_dependencies(head_focus_node
  rclcpp
  bodyctrl_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

# Устанавливаем ноды, чтобы их можно было запустить через ros2 run
install(TARGETS
  zahvat_node
  head_demo_node
  head_focus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
