#ifndef ARM_CONTROL_HPP_
#define ARM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace arm_control
{

// State machine states
enum class ArmState {
  IDLE,
  LEFT_GROUND_GRABBING,
  RIGHT_GROUND_GRABBING,
  HARVESTING
};

class ArmControlNode : public rclcpp::Node
{
public:
  ArmControlNode();
  virtual ~ArmControlNode() = default;

private:
  // 初始化MoveIt接口
  void initializeMoveIt();
  
  // Timer callback for state machine loop
  void timerCallback();
  
  // Timer callback for servo angle publishing
  void publishServoAnglesCallback();
  
  // State machine handlers
  void handleIdleState();
  void handleLeftGroundGrabbing();
  void handleRightGroundGrabbing();
  void handleHarvesting();
  
  // Transition functions
  void transitionToIdle();
  void transitionToLeftGroundGrabbing();
  void transitionToRightGroundGrabbing();
  void transitionToHarvesting();

  // Joint state callback
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Target offset callback
  void targetOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  
  // Data init callback for receiving data from serial port
  void dataInitCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // MoveIt related members
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  
  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_offset_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr data_init_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_angle_pub_;
  
  // Latest joint positions
  std::vector<double> latest_joint_positions_;
  std::mutex joint_positions_mutex_;
  
  // Target offset data
  float target_x_offset_;
  float target_y_offset_;
  int target_class_id_;
  std::mutex target_offset_mutex_;
  bool target_detected_;
  
  // Data received from serial communication
  std::vector<float> serial_data_;
  std::mutex serial_data_mutex_;
  
  // Current state
  ArmState current_state_;

  // Current pose
  geometry_msgs::msg::Pose current_pose_;
  std::mutex current_pose_mutex_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr servo_publish_timer_;
  rclcpp::TimerBase::SharedPtr move_group_init_timer_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
  
  // Parameters
  double loop_rate_;
};

} // namespace arm_control

#endif // ARM_CONTROL_HPP_ 