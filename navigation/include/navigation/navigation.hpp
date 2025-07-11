#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <navigation/action/move.hpp>
#include <navigation/msg/move_msg.hpp>
#include <memory>
#include <string>

namespace navigation
{

class MovementActionServer : public rclcpp::Node
{
public:
  MovementActionServer();
  ~MovementActionServer() = default;

  // 初始化函数
  void initialize();

private:
  using MoveAction = navigation::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveAction>;
  using MoveMsg = navigation::msg::MoveMsg;
  
  // 动作服务器
  rclcpp_action::Server<MoveAction>::SharedPtr move_action_server_;
  
  // 移动命令发布器
  rclcpp::Publisher<MoveMsg>::SharedPtr move_msg_publisher_;
  
  // 动作服务器回调函数
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveAction::Goal> goal);
  
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle);
  
  void handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle);
    
  // 执行线程
  void execute_movement(
    const std::shared_ptr<GoalHandleMove> goal_handle);
  
  // 辅助函数
  void move_linear(float distance, const std::shared_ptr<GoalHandleMove> goal_handle);
  void move_angular(float angle, const std::shared_ptr<GoalHandleMove> goal_handle);
  
  // 日志辅助函数
  void logInfo(const std::string& message);
  void logError(const std::string& message);
  void logWarning(const std::string& message);
};

} // namespace navigation

#endif // NAVIGATION_HPP_ 