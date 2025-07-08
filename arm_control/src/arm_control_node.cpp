#include "arm_control/arm_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 12); // 使用8个线程
  
  // 创建节点
  auto node = std::make_shared<arm_control::ArmControlNode>();
  // 添加节点到执行器
  executor.add_node(node);
  
  std::cout << "Starting executor spin..." << std::endl;
  
  // 运行执行器
  std::thread executor_thread([&]() {
    executor.spin();
  });
  
  executor_thread.join();
  
  rclcpp::shutdown();
  return 0;
} 