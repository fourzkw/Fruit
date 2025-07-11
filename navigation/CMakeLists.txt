cmake_minimum_required(VERSION 3.8)
project(navigation)

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
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action and message definitions
set(action_files
  "action/Move.action"
)

set(msg_files
  "msg/MoveMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

# Build navigation_node
add_executable(navigation_node
  src/navigation_node.cpp
  src/navigation.cpp
)

# Add dependencies to target
ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_action
  geometry_msgs
)

rosidl_target_interfaces(navigation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  msg
  DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
