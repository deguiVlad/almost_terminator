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
find_package(bodyctrl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

install(PROGRAMS
  scripts/object_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- launch + config ------------------------------------------------------

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
