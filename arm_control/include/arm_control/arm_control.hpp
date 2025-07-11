#ifndef ARM_CONTROL_HPP_
#define ARM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <atomic>
#include "arm_control/action/grab.hpp"

namespace arm_control
{

class ArmControlNode : public rclcpp::Node
{
public:
  ArmControlNode();
  ~ArmControlNode();

private:
  using GrabAction = arm_control::action::Grab;
  using GoalHandleGrab = rclcpp_action::ServerGoalHandle<GrabAction>;

  // 核心方法
  void initializeMoveIt();
  bool waitForJointPositionConvergence(double max_error, int timeout_ms);
  bool moveEndEffectorCartesian(double x_displacement, double y_displacement, double z_displacement, int timeout_ms);
  
  // 状态处理器
  bool handleLeftGroundGrabbing();
  bool handleRightGroundGrabbing();
  bool handleHarvesting();
  
  // 状态转换
  bool transitionToIdle();
  bool transitionToLeftGroundGrabbing();
  bool transitionToRightGroundGrabbing();
  bool transitionToHarvesting();

  // 回调函数
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void dataInitCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // 动作服务器回调
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GrabAction::Goal> goal);
  
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGrab> goal_handle);
  
  void handleAccepted(const std::shared_ptr<GoalHandleGrab> goal_handle);
  
  void executeGrabAction(const std::shared_ptr<GoalHandleGrab> goal_handle);
  
  // 线程相关
  std::thread servo_publish_thread_;
  std::atomic<bool> servo_publish_thread_running_;
  void servoPublishThreadFunc();

  // MoveIt组件
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  
  // ROS通信
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_offset_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr data_init_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr garb_msg_pub_;
  
  // 动作服务器
  rclcpp_action::Server<GrabAction>::SharedPtr grab_action_server_;
  
  // 关节数据
  std::vector<double> latest_joint_positions_;
  std::mutex joint_positions_mutex_;
  
  // 目标数据
  float target_x_offset_;
  float target_y_offset_;
  int target_class_id_;
  std::mutex target_offset_mutex_;
  bool target_detected_;
  
  // 串口数据
  std::vector<float> serial_data_;  //1-4为四个舵机角度，5-8为四个传感器值，9为imu_yaw
  std::mutex serial_data_mutex_;
  
  // 状态跟踪
  geometry_msgs::msg::Pose current_pose_;
  std::mutex current_pose_mutex_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr move_group_init_timer_;
  
  // 机械爪状态 (0 = 打开, 1 = 关闭)
  float gripper_state_;
  std::mutex gripper_state_mutex_;
};

} // namespace arm_control

#endif // ARM_CONTROL_HPP_ 