#include "arm_control/arm_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 使用4个线程
  
  // 创建节点
  auto node = std::make_shared<arm_control::ArmControlNode>();
  
  RCLCPP_INFO(node->get_logger(), "arm_control_node initialized with multi-threaded executor");
  
  // 添加节点到执行器
  executor.add_node(node);
  
  // 运行执行器
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 