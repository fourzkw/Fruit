#ifndef TASK_PLANNER_HPP_
#define TASK_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <arm_control/action/grab.hpp>
#include <memory>
#include <string>
#include <vector>

namespace task_planner
{

class TaskPlannerNode : public rclcpp::Node
{
public:
  TaskPlannerNode();
  ~TaskPlannerNode() = default;

  // 初始化函数
  void initialize();
  
  // 发送抓取目标的函数
  bool sendGrabGoal(int direction, const std::vector<int>& types, int position);
  
  // 取消当前抓取目标
  bool cancelCurrentGoal();

private:
  using GrabAction = arm_control::action::Grab;
  using GoalHandleGrab = rclcpp_action::ClientGoalHandle<GrabAction>;
  
  // 动作客户端
  rclcpp_action::Client<GrabAction>::SharedPtr grab_action_client_;
  
  // 当前目标处理
  std::shared_ptr<GoalHandleGrab> current_goal_handle_;
  bool goal_in_progress_;
  
  // 动作客户端回调函数
  void goal_response_callback(std::shared_ptr<GoalHandleGrab> goal_handle);
  void feedback_callback(GoalHandleGrab::SharedPtr goal_handle,
                         const std::shared_ptr<const GrabAction::Feedback> feedback);
  void result_callback(const GoalHandleGrab::WrappedResult& result);
  
  // 定时器和回调
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  
  // 日志辅助函数
  void logInfo(const std::string& message);
  void logError(const std::string& message);
  void logWarning(const std::string& message);
};

} // namespace task_planner

#endif // TASK_PLANNER_HPP_ 