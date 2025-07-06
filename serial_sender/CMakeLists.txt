cmake_minimum_required(VERSION 3.8)
project(portable_serial_sender)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)

# 创建库目标
add_library(${PROJECT_NAME} SHARED
  src/serial_sender.cpp
  src/struct_serializer.cpp
)

# 设置头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  serial_driver
  std_msgs
)

# 创建示例可执行文件
add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node ${PROJECT_NAME})
ament_target_dependencies(serial_node
  rclcpp
  serial_driver
  std_msgs
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装可执行文件
install(TARGETS serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出目标
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp serial_driver std_msgs)

# 启用作为独立包使用
ament_package() 