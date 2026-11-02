# cmake_minimum_required
cmake_minimum_required(VERSION 3.8)

# cmake project
project(task_2)

# cmake compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cmake find dependencies
find_package(ament_cmake REQUIRED)
find_package(bodyctrl_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# cmake build executable
add_executable(move_head_publisher src/move_head_publisher.cpp)
ament_target_dependencies(move_head_publisher
  ament_cmake
  bodyctrl_msgs
  std_msgs
  builtin_interfaces
)


