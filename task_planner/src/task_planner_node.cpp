#include "task_planner/task_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

// 简单测试函数，发送抓取请求
void test_send_grab_requests(std::shared_ptr<task_planner::TaskPlannerNode> node)
{
  // 等待10秒，确保arm_control节点完全初始化
  std::this_thread::sleep_for(std::chrono::seconds(10));
  
  // 水果种类数组
  std::vector<int> fruit_types = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};  // 水果种类1-11
  
  // 发送左侧地面抓取请求 (direction=0, types=fruit_types, position=0 表示地面)
  int position = 0;  // 0: 地面, 1: 树上
  node->sendGrabGoal(0, fruit_types, position);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建任务规划器节点
  auto node = std::make_shared<task_planner::TaskPlannerNode>();
  
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // 启动测试线程发送抓取请求
  std::thread test_thread(test_send_grab_requests, node);
  test_thread.detach();
  
  std::cout << "任务规划器节点已启动，等待命令..." << std::endl;
  
  // 启动执行器
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 