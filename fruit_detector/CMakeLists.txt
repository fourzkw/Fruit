cmake_minimum_required(VERSION 3.8)
project(fruit_detector)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime
  PATHS /usr/lib/cmake/openvino2025.2.0
  NO_DEFAULT_PATH
)

# Include directories
include_directories(include)

# Create the fruit detector library
add_library(fruit_detector_component SHARED
  src/fruit_detector.cpp
  src/openvino_detect.cpp
)
target_compile_definitions(fruit_detector_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(fruit_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  vision_msgs
  OpenCV
  OpenVINO
)
# Explicitly link OpenVINO libraries
target_link_libraries(fruit_detector_component
  openvino::runtime
)
rclcpp_components_register_nodes(fruit_detector_component "fruit_detector::FruitDetector")

# Create the standalone executable
add_executable(fruit_detector_node
  src/fruit_detector_node.cpp
)
target_link_libraries(fruit_detector_node
  fruit_detector_component
)
ament_target_dependencies(fruit_detector_node
  rclcpp
)

# Install targets
install(TARGETS
  fruit_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  fruit_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 