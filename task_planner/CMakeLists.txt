cmake_minimum_required(VERSION 3.8)
project(task_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arm_control REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build task_planner_node
add_executable(task_planner_node
  src/task_planner_node.cpp
  src/task_planner.cpp
)

ament_target_dependencies(task_planner_node
  rclcpp
  rclcpp_action
  arm_control
  geometry_msgs
)

# Install
install(TARGETS
  task_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
