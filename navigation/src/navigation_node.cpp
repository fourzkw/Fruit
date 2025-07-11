#include "navigation/navigation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建导航服务器节点
  auto node = std::make_shared<navigation::MovementActionServer>();
  
  std::cout << "导航节点已启动，等待移动指令..." << std::endl;
  
  // 启动执行器
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
} 