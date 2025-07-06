#include "arm_control/arm_control.hpp"

namespace arm_control
{

ArmControlNode::ArmControlNode()
: Node("arm_control_node"),
  current_state_(ArmState::IDLE),
  loop_rate_(5), // 状态机频率
  target_detected_(false),
  target_x_offset_(0.0f),
  target_y_offset_(0.0f),
  target_class_id_(0),
  serial_data_(9, 0.0f),
  idle_timer_(nullptr)
{
  // 声明参数
  this->declare_parameter("loop_rate", loop_rate_);
  
  // 获取参数
  loop_rate_ = this->get_parameter("loop_rate").as_double();

  // 初始化MoveIt
  // 注意：使用回调组在单独的线程中处理MoveIt初始化
  // 因为它可能需要一些时间并阻塞节点初始化
  auto moveit_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
    
  // 创建一个并行回调组，用于关节状态更新
  auto joint_state_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  
  // 创建并行回调组用于伺服角度发布
  auto servo_publish_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
    
  // 创建并行回调组用于目标偏移订阅
  auto target_offset_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
    
  // 创建并行回调组用于串口数据订阅
  auto serial_data_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  
  // 创建状态机循环的定时器
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / loop_rate_),
    std::bind(&ArmControlNode::timerCallback, this));
  
  // 创建100ms的伺服角度发布定时器，使用并行回调组
  rclcpp::TimerOptions servo_timer_options;
  servo_timer_options.callback_group = servo_publish_callback_group;
  servo_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ArmControlNode::publishServoAnglesCallback, this),
    servo_timer_options);
  
  // 创建关节状态订阅者，使用并行回调组
  rclcpp::SubscriptionOptions joint_state_options;
  joint_state_options.callback_group = joint_state_callback_group;
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 
    rclcpp::QoS(10).reliable(), 
    std::bind(&ArmControlNode::jointStateCallback, this, std::placeholders::_1),
    joint_state_options);
  
  // 创建目标偏移订阅者，使用并行回调组
  rclcpp::SubscriptionOptions target_offset_options;
  target_offset_options.callback_group = target_offset_callback_group;
  target_offset_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/target_offset", 10,
    std::bind(&ArmControlNode::targetOffsetCallback, this, std::placeholders::_1),
    target_offset_options);
  
  // 创建串口数据订阅者，使用并行回调组
  rclcpp::SubscriptionOptions serial_data_options;
  serial_data_options.callback_group = serial_data_callback_group;
  data_init_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/data_init", 10,
    std::bind(&ArmControlNode::dataInitCallback, this, std::placeholders::_1),
    serial_data_options);
  
  // 创建舵机角度发布者
  servo_angle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/servo_angle", 10);
  
  // 创建延迟初始化MoveIt接口的一次性定时器
  // 在节点完全初始化后执行，避免在构造函数中使用shared_from_this()
  move_group_init_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    [this]() {
      this->initializeMoveIt();
      // 初始化完成后取消定时器
      move_group_init_timer_->cancel();
    });
}

// 初始化MoveIt接口的方法
void ArmControlNode::initializeMoveIt()
{
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "group1");
    // 设置规划时长为2秒
    move_group_->setPlanningTime(2.0);
    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized successfully with planning group 'group1'");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interface: %s", e.what());
  }
}

void ArmControlNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 更新最新的关节位置
  std::lock_guard<std::mutex> lock(joint_positions_mutex_);
  latest_joint_positions_.resize(msg->position.size());
  
  for (size_t i = 0; i < msg->position.size(); ++i) {
    latest_joint_positions_[i] = msg->position[i];
  }
}

// Data init callback 处理从串口接收到的数据
void ArmControlNode::dataInitCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // 检查消息中的数据是否符合预期
  if (msg->data.size() == 9) {
    // 线程安全地更新数据
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    
    // 复制数据到成员变量
    for (size_t i = 0; i < 9; ++i) {
      serial_data_[i] = msg->data[i];
    }
    
    RCLCPP_DEBUG(this->get_logger(), "接收到串口数据: "
      "servo1=%.2f, servo2=%.2f, servo3=%.2f, servo4=%.2f, "
      "encoder1=%.2f, encoder2=%.2f, encoder3=%.2f, encoder4=%.2f, "
      "imu_yaw=%.2f",
      serial_data_[0], serial_data_[1], serial_data_[2], serial_data_[3],
      serial_data_[4], serial_data_[5], serial_data_[6], serial_data_[7],
      serial_data_[8]);
  } else {
    RCLCPP_WARN(this->get_logger(), "接收到的数据大小不符合预期: %zu, 预期 9", msg->data.size());
  }
}

void ArmControlNode::publishServoAnglesCallback()
{
  // 创建Float32MultiArray消息并发布最新的关节角度数据
  std_msgs::msg::Float32MultiArray servo_angles;
  
  {
    // 线程安全地访问最新的关节位置
    std::lock_guard<std::mutex> lock(joint_positions_mutex_);
    
    // 如果还没有接收到关节数据，不发布
    if (latest_joint_positions_.empty()) {
      return;
    }
    
    // 将关节角度数据复制到Float32MultiArray消息
    servo_angles.data.resize(latest_joint_positions_.size());
    for (size_t i = 0; i < latest_joint_positions_.size(); ++i) {
      servo_angles.data[i] = static_cast<float>(latest_joint_positions_[i]);
    }
  }
  
  // 发布伺服角度消息
  servo_angle_pub_->publish(servo_angles);
  
  RCLCPP_DEBUG(this->get_logger(), "Published %zu joint angles to /servo_angle", servo_angles.data.size());
}

void ArmControlNode::timerCallback()
{
  // 主状态机循环
  switch (current_state_) {
    // 待机状态
    case ArmState::IDLE:
      handleIdleState();
      break;
    // 左侧地面抓取状态
    case ArmState::LEFT_GROUND_GRABBING:
      handleLeftGroundGrabbing();
      break;
    // 右侧地面抓取状态
    case ArmState::RIGHT_GROUND_GRABBING:
      handleRightGroundGrabbing();
      break;
    // 收获状态
    case ArmState::HARVESTING:
      handleHarvesting();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown state in state machine");
      // 如果遇到未知状态，回到IDLE状态
      transitionToIdle();
      break;
  }
}

void ArmControlNode::handleIdleState()
{
  // 如果定时器已经被创建并且是激活状态，则不需要重新创建
  if (idle_timer_ && idle_timer_->is_steady()) {
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Handling IDLE state - waiting 5 seconds before transitioning");
  
  // 先取消之前可能存在的定时器
  if (idle_timer_) {
    idle_timer_->cancel();
  }
  
  // 创建一次性定时器，等待5秒后切换到左侧地面抓取状态
  idle_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      RCLCPP_INFO(this->get_logger(), "5-second wait completed, transitioning to left ground grabbing");
      this->transitionToLeftGroundGrabbing();
      // 一次性定时器，触发后自动取消
      idle_timer_->cancel();
    });
}

void ArmControlNode::handleLeftGroundGrabbing()
{
  RCLCPP_DEBUG(this->get_logger(), "Handling LEFT_GROUND_GRABBING state");
  
  // 检测逻辑和其他操作可以保留在这里
  
  // 完成后转换到收获状态
  // transitionToHarvesting();
}

void ArmControlNode::handleRightGroundGrabbing()
{
  RCLCPP_DEBUG(this->get_logger(), "Handling RIGHT_GROUND_GRABBING state");
  
  /* 实现将包括：
   * 1. 将机械臂移动到适合右侧地面的位置
   * 2. 激活相机和检测算法
   * 3. 处理检测结果
   * 4. 规划并执行运动以抓取检测到的水果
   * 5. 控制夹持器拾取水果
   * 6. 转换到下一个状态
   */
  
  // 占位符：为了演示，抓取后转换到收获状态
  transitionToHarvesting();
}

void ArmControlNode::handleHarvesting()
{
  RCLCPP_DEBUG(this->get_logger(), "Handling HARVESTING state");
  
  /* 实现将包括：
   * 1. 将机械臂移动到收集箱以存放水果
   * 2. 释放夹持器
   * 3. 记录收获的水果
   * 4. 转换到下一个状态
   */
  
  // 占位符：为了演示，收获后转换到右侧抓取状态
  // 如果从左侧抓取过来，就转到右侧抓取；如果从右侧抓取过来，就回到空闲状态
  static bool from_left = true;
  
  if (from_left) {
    from_left = false;
    transitionToRightGroundGrabbing();
  } else {
    from_left = true;
    transitionToIdle();
  }
}

// 状态转换函数
void ArmControlNode::transitionToIdle()
{
  RCLCPP_INFO(this->get_logger(), "Transitioning to IDLE state");
  current_state_ = ArmState::IDLE;
  
  try {
    // 检查MoveIt接口是否已初始化
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized");
      return;
    }
    
    // 设置机械臂的目标位置为"mid_pose"
    move_group_->setNamedTarget("mid_pose");
    
    // 规划运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    
    // 检查规划是否成功
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Planning to mid_pose failed!");
      return;
    }
    
    move_group_->execute(plan);
    
    // 获取当前关节角度（使用订阅的关节状态）
    std::vector<double> joint_values;
    {
      std::lock_guard<std::mutex> lock(joint_positions_mutex_);
      joint_values = latest_joint_positions_;
    }
    
    // 显示每个关节的角度
    RCLCPP_INFO(this->get_logger(), "Current joint angles:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %.4f rad (%.2f deg)", 
                  i+1, joint_values[i], joint_values[i] * 180.0 / M_PI);
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in transitioning to idle: %s", e.what());
  }
}

void ArmControlNode::transitionToLeftGroundGrabbing()
{ 
  
  current_state_ = ArmState::LEFT_GROUND_GRABBING;
  // 尝试将机械臂移动到预定义的左侧位置
  try {
    // 检查MoveIt接口是否已初始化
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized");
      return;
    }
    
    // 设置机械臂的目标位置为"left_pose"
    move_group_->setNamedTarget("left_pose");
    
    // 规划运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    
    // 检查规划是否成功
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Planning to left_pose failed!");
      return;
    }
    
    move_group_->execute(plan);
    
    // 获取当前关节角度
    std::vector<double> joint_values;
    {
      std::lock_guard<std::mutex> lock(joint_positions_mutex_);
      joint_values = latest_joint_positions_;
    }
    
    // 显示每个关节的角度
    RCLCPP_INFO(this->get_logger(), "Current joint angles:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %.4f rad (%.2f deg)", 
                  i+1, joint_values[i], joint_values[i] * 180.0 / M_PI);
    }
    
    // 等待舵机实际到达规划位置
    const double angle_tolerance = 0.05; // 角度误差容忍范围（弧度）
    const int max_wait_cycles = 100; // 最大等待循环次数
    int wait_cycles = 0;
    
    RCLCPP_INFO(this->get_logger(), "等待舵机到达目标位置...");
    
    while(wait_cycles < max_wait_cycles)
    {
      // 获取下位机上传的实际舵机角度
      std::vector<float> current_servo_angles;
      {
        std::lock_guard<std::mutex> lock(serial_data_mutex_);
        // 只取前4个值，它们是舵机角度
        current_servo_angles = {serial_data_[0], serial_data_[1], 
                               serial_data_[2], serial_data_[3]};
      }
      
      // 检查是否有足够的关节数据进行比较
      if (joint_values.size() < 4 || current_servo_angles.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "没有足够的关节数据进行比较，等待下一次数据更新");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_cycles++;
        continue;
      }
      
      // 检查每个关节的误差
      bool all_joints_in_position = true;
      for (size_t i = 0; i < 4; ++i) {
        // 将规划角度从弧度转换为与serial_data_相同的单位（通常是度）
        double planned_angle_deg = joint_values[i] * 180.0 / M_PI;
        double actual_angle_deg = current_servo_angles[i];
        
        // 计算误差
        double error = std::abs(planned_angle_deg - actual_angle_deg);
        
        // 如果任何一个关节超出误差范围，标记为未就位
        if (error > angle_tolerance * 180.0 / M_PI) { // 转换容忍度为度
          all_joints_in_position = false;
          RCLCPP_DEBUG(this->get_logger(), "关节 %zu 误差: %.2f 度 (规划:%.2f, 实际:%.2f)", 
                        i+1, error, planned_angle_deg, actual_angle_deg);
          break;
        }
      }
      
      // 如果所有关节都在位置上，退出循环
      if (all_joints_in_position) {
        RCLCPP_INFO(this->get_logger(), "所有舵机已达到目标位置");
        break;
      }
      
      // 等待一段时间再检查
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wait_cycles++;
    }
    
    // 检查是否因超时退出
    if (wait_cycles >= max_wait_cycles) {
      RCLCPP_WARN(this->get_logger(), "等待舵机到位超时");
      transitionToIdle();
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in left ground grabbing: %s", e.what());
    transitionToIdle();
  }
}

void ArmControlNode::transitionToRightGroundGrabbing()
{
  current_state_ = ArmState::RIGHT_GROUND_GRABBING;
}

void ArmControlNode::transitionToHarvesting()
{
  RCLCPP_INFO(this->get_logger(), "Transitioning to HARVESTING state");
  current_state_ = ArmState::HARVESTING;
  
  // 可以在这里实现额外的转换动作
}

void ArmControlNode::targetOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // 线程安全地更新目标偏移数据
  {
    std::lock_guard<std::mutex> lock(target_offset_mutex_);
    target_x_offset_ = msg->x;
    target_y_offset_ = msg->y;
    target_class_id_ = static_cast<int>(msg->z);
    target_detected_ = true;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Received target offset: x=%.2f, y=%.2f, class_id=%d", 
              target_x_offset_, target_y_offset_, target_class_id_);
}

} // namespace arm_control 