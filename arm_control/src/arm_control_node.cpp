#include "arm_control/arm_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建多线程执行器，增加线程数以确保能处理所有回调组
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 16); // 使用16个线程
  
  // 创建节点
  auto node = std::make_shared<arm_control::ArmControlNode>();
  // 添加节点到执行器
  executor.add_node(node);
  
  std::cout << "正在启动arm_control节点..." << std::endl;
  
  // 运行执行器
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 