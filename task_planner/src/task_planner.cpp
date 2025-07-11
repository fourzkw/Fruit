#include "task_planner/task_planner.hpp"
#include <functional>
#include <future>
#include <chrono>

namespace task_planner
{

TaskPlannerNode::TaskPlannerNode()
  : Node("task_planner_node"), grab_goal_in_progress_(false), move_goal_in_progress_(false)
{
  // 节点构造时进行初始化
  initialize();
}

void TaskPlannerNode::initialize()
{
  // 创建抓取动作客户端
  grab_action_client_ = rclcpp_action::create_client<GrabAction>(
    this, "/grab");
  
  // 等待抓取动作服务器可用
  if (!grab_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    logError("抓取动作服务器不可用，请确保arm_control节点已运行");
  } else {
    logInfo("已连接到抓取动作服务器");
  }
  
  // 创建移动动作客户端
  move_action_client_ = rclcpp_action::create_client<MoveAction>(
    this, "/move");
  
  // 等待移动动作服务器可用
  if (!move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    logError("移动动作服务器不可用，请确保navigation节点已运行");
  } else {
    logInfo("已连接到移动动作服务器");
  }
  
  // 创建定时器用于演示目的
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TaskPlannerNode::timer_callback, this));
}

bool TaskPlannerNode::sendGrabGoal(int direction, const std::vector<int>& types, int position)
{
  // 如果动作客户端未准备好，返回失败
  if (!grab_action_client_) {
    logError("抓取动作客户端未初始化");
    return false;
  }
  
  // 如果已有目标在进行中，返回失败
  if (grab_goal_in_progress_) {
    logWarning("已有抓取任务正在进行中，请先取消当前任务");
    return false;
  }
  
  // 创建目标消息
  auto goal_msg = GrabAction::Goal();
  goal_msg.direction = direction;
  
  // 将水果类型数组转换为int8[]
  goal_msg.type_array.resize(types.size());
  for (size_t i = 0; i < types.size(); ++i) {
    goal_msg.type_array[i] = static_cast<int8_t>(types[i]);
  }
  
  goal_msg.position = position;
  
  // 构建水果类型日志信息
  std::string types_str = "[";
  for (size_t i = 0; i < types.size(); ++i) {
    types_str += std::to_string(types[i]);
    if (i < types.size() - 1) {
      types_str += ", ";
    }
  }
  types_str += "]";
  
  logInfo("发送抓取请求: 方向=" + std::to_string(direction) + 
          ", 类型数组=" + types_str + 
          ", 位置=" + std::to_string(position) + "(0:地面,1:树上)");
  
  // 设置目标发送选项
  auto send_goal_options = rclcpp_action::Client<GrabAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TaskPlannerNode::grab_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&TaskPlannerNode::grab_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&TaskPlannerNode::grab_result_callback, this, std::placeholders::_1);
  
  // 发送目标
  grab_goal_in_progress_ = true;
  auto goal_handle_future = grab_action_client_->async_send_goal(goal_msg, send_goal_options);
  
  return true;
}

bool TaskPlannerNode::sendMoveGoal(int movement_type, float value)
{
  // 如果动作客户端未准备好，返回失败
  if (!move_action_client_) {
    logError("移动动作客户端未初始化");
    return false;
  }
  
  // 如果已有目标在进行中，返回失败
  if (move_goal_in_progress_) {
    logWarning("已有移动任务正在进行中，请先取消当前任务");
    return false;
  }
  
  // 创建目标消息
  auto goal_msg = MoveAction::Goal();
  goal_msg.movement_type = movement_type;
  goal_msg.value = value;
  
  // 打印移动类型
  std::string movement_str = (movement_type == 0) ? "直线移动" : "角度旋转";
  std::string value_str = (movement_type == 0) ? " 米" : " 弧度";
  
  logInfo("发送移动请求: 类型=" + movement_str + 
          ", 值=" + std::to_string(value) + value_str);
  
  // 设置目标发送选项
  auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TaskPlannerNode::move_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&TaskPlannerNode::move_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&TaskPlannerNode::move_result_callback, this, std::placeholders::_1);
  
  // 发送目标
  move_goal_in_progress_ = true;
  auto goal_handle_future = move_action_client_->async_send_goal(goal_msg, send_goal_options);
  
  return true;
}

bool TaskPlannerNode::cancelCurrentGrabGoal()
{
  if (!grab_goal_in_progress_ || !current_grab_goal_handle_) {
    logWarning("没有正在进行中的抓取目标可以取消");
    return false;
  }
  
  logInfo("正在取消当前抓取目标");
  auto cancel_future = grab_action_client_->async_cancel_goal(current_grab_goal_handle_);
  
  return true;
}

bool TaskPlannerNode::cancelCurrentMoveGoal()
{
  if (!move_goal_in_progress_ || !current_move_goal_handle_) {
    logWarning("没有正在进行中的移动目标可以取消");
    return false;
  }
  
  logInfo("正在取消当前移动目标");
  auto cancel_future = move_action_client_->async_cancel_goal(current_move_goal_handle_);
  
  return true;
}

void TaskPlannerNode::grab_goal_response_callback(std::shared_ptr<GoalHandleGrab> goal_handle)
{
  if (!goal_handle) {
    logError("抓取请求被拒绝");
    grab_goal_in_progress_ = false;
  } else {
    logInfo("抓取请求被接受");
    current_grab_goal_handle_ = goal_handle;
  }
}

void TaskPlannerNode::grab_feedback_callback(
  GoalHandleGrab::SharedPtr goal_handle,
  const std::shared_ptr<const GrabAction::Feedback> feedback)
{
  (void)goal_handle;  // 未使用的参数
  logInfo("收到抓取反馈：完成度 " + std::to_string(feedback->completion_percentage) + "%");
}

void TaskPlannerNode::grab_result_callback(const GoalHandleGrab::WrappedResult& result)
{
  grab_goal_in_progress_ = false;
  current_grab_goal_handle_ = nullptr;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      logInfo("抓取任务完成，结果: " + std::string(result.result->success ? "成功" : "失败") + 
              ", 消息: " + result.result->message);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      logError("抓取任务被终止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      logWarning("抓取任务被取消");
      break;
    default:
      logError("未知结果代码");
      break;
  }
}

void TaskPlannerNode::move_goal_response_callback(std::shared_ptr<GoalHandleMove> goal_handle)
{
  if (!goal_handle) {
    logError("移动请求被拒绝");
    move_goal_in_progress_ = false;
  } else {
    logInfo("移动请求被接受");
    current_move_goal_handle_ = goal_handle;
  }
}

void TaskPlannerNode::move_feedback_callback(
  GoalHandleMove::SharedPtr goal_handle,
  const std::shared_ptr<const MoveAction::Feedback> feedback)
{
  (void)goal_handle;  // 未使用的参数
  logInfo("收到移动反馈：完成度 " + std::to_string(feedback->completion_percentage) + "%");
}

void TaskPlannerNode::move_result_callback(const GoalHandleMove::WrappedResult& result)
{
  move_goal_in_progress_ = false;
  current_move_goal_handle_ = nullptr;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      logInfo("移动任务完成，结果: " + std::string(result.result->success ? "成功" : "失败") + 
              ", 消息: " + result.result->message);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      logError("移动任务被终止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      logWarning("移动任务被取消");
      break;
    default:
      logError("未知结果代码");
      break;
  }
}

void TaskPlannerNode::timer_callback()
{
  // 此函数为演示目的，可以在这里实现定期检查或处理逻辑
  // 例如，检查是否有新的目标需要发送
}

// 日志辅助函数实现
void TaskPlannerNode::logInfo(const std::string& message)
{
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

void TaskPlannerNode::logError(const std::string& message)
{
  RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
}

void TaskPlannerNode::logWarning(const std::string& message)
{
  RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
}

} // namespace task_planner 