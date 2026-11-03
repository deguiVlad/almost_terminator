cmake_minimum_required(VERSION 3.8)
project(focus_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(bodyctrl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- собственные srv ------------------------------------------------------
# DetectObjects: ручной список обнаруженных объектов + выбор цели по id/имени.

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  DEPENDENCIES geometry_msgs
)

# --- C++ ноды -------------------------------------------------------------

# Источник цели «рука» из TF2 (кинематика, без зрения)
add_executable(hand_tracker_node src/hand_tracker_node.cpp)
ament_target_dependencies(hand_tracker_node
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs tf2_ros tf2_geometry_msgs)

# Универсальный контроллер головы: object | hand | both
add_executable(focus_controller_node src/focus_controller_node.cpp)
ament_target_dependencies(focus_controller_node
  rclcpp geometry_msgs sensor_msgs std_srvs bodyctrl_msgs)

install(TARGETS
  hand_tracker_node
  focus_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Python ноды ----------------------------------------------------------
# object_detector_node.py использует сгенерированный srv-модуль этого пакета,
# поэтому должен быть собран в PYTHONPATH после rosidl_generate_interfaces.

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hand_tracker_node "${cpp_typesupport_target}")

install(PROGRAMS
  scripts/object_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# запускной скрипт-обёртка не нужен: srv доступны через ament_python_index (см. package.xml)

# --- launch + config ------------------------------------------------------

install(DIRECTORY launch config srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
