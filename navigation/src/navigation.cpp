#include "navigation/navigation.hpp"
#include <functional>
#include <future>
#include <chrono>
#include <thread>

namespace navigation
{

MovementActionServer::MovementActionServer()
  : Node("navigation_node")
{
  // 节点构造时进行初始化
  initialize();
}

void MovementActionServer::initialize()
{
  // 创建动作服务器
  this->move_action_server_ = rclcpp_action::create_server<MoveAction>(
    this,
    "/move",
    std::bind(&MovementActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MovementActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MovementActionServer::handle_accepted, this, std::placeholders::_1));
  
  // 创建移动命令发布器
  move_msg_publisher_ = this->create_publisher<MoveMsg>("/move_msg", 10);
    
  logInfo("导航动作服务器已启动，等待移动指令");
}

rclcpp_action::GoalResponse MovementActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveAction::Goal> goal)
{
  (void)uuid; // 未使用的参数
  
  // 验证移动类型
  if (goal->movement_type != 0 && goal->movement_type != 1) {
    logError("无效的移动类型: " + std::to_string(goal->movement_type) + 
             "，应为 0(直线移动) 或 1(角度旋转)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  // 创建并发布移动消息
  auto move_msg = MoveMsg();
  
  // 根据移动类型设置目标值
  if (goal->movement_type == 0) {
    // 直线移动
    move_msg.target_distance = goal->value;
    move_msg.target_angle = 0.0;
    logInfo("收到直线移动请求，距离: " + std::to_string(goal->value) + " 米");
  } else {
    // 角度旋转
    move_msg.target_distance = 0.0;
    move_msg.target_angle = goal->value;
    logInfo("收到角度旋转请求，角度: " + std::to_string(goal->value) + " 弧度");
  }
  
  // 发布移动消息
  move_msg_publisher_->publish(move_msg);
  logInfo("已将移动命令发布到/move_msg话题: 目标距离=" + std::to_string(move_msg.target_distance) +
          ", 目标角度=" + std::to_string(move_msg.target_angle));
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MovementActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleMove> goal_handle)
{
  (void)goal_handle; // 未使用的参数
  logWarning("收到取消移动请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MovementActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleMove> goal_handle)
{
  // 在新线程中执行任务
  std::thread{std::bind(&MovementActionServer::execute_movement, this, goal_handle)}.detach();
}

void MovementActionServer::execute_movement(
  const std::shared_ptr<GoalHandleMove> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveAction::Feedback>();
  auto result = std::make_shared<MoveAction::Result>();
  
  // 根据移动类型选择操作
  if (goal->movement_type == 0) {
    // 直线移动
    move_linear(goal->value, goal_handle);
  } else {
    // 角度旋转
    move_angular(goal->value, goal_handle);
  }
  
  // 检查任务是否被取消
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "移动任务被取消";
    goal_handle->canceled(result);
    logWarning("移动任务被取消");
    return;
  }
  
  // 任务完成
  result->success = true;
  result->message = "移动任务完成";
  goal_handle->succeed(result);
  logInfo("移动任务成功完成");
}

void MovementActionServer::move_linear(float distance, const std::shared_ptr<GoalHandleMove> goal_handle)
{
  auto feedback = std::make_shared<MoveAction::Feedback>();
  
  // 模拟线性移动执行过程
  for (int i = 0; i <= 10; i++) {
    // 检查任务是否被取消
    if (goal_handle->is_canceling()) {
      return;
    }
    
    // 更新进度
    feedback->completion_percentage = i * 10.0f;
    goal_handle->publish_feedback(feedback);
    logInfo("直线移动进度: " + std::to_string(i * 10.0f) + "%");
    
    // 模拟移动时间
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void MovementActionServer::move_angular(float angle, const std::shared_ptr<GoalHandleMove> goal_handle)
{
  auto feedback = std::make_shared<MoveAction::Feedback>();
  
  // 模拟角度旋转执行过程
  for (int i = 0; i <= 10; i++) {
    // 检查任务是否被取消
    if (goal_handle->is_canceling()) {
      return;
    }
    
    // 更新进度
    feedback->completion_percentage = i * 10.0f;
    goal_handle->publish_feedback(feedback);
    logInfo("角度旋转进度: " + std::to_string(i * 10.0f) + "%");
    
    // 模拟旋转时间
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

// 日志辅助函数实现
void MovementActionServer::logInfo(const std::string& message)
{
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

void MovementActionServer::logError(const std::string& message)
{
  RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
}

void MovementActionServer::logWarning(const std::string& message)
{
  RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
}

} // namespace navigation 