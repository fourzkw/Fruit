#include "arm_control/arm_control.hpp"
#include <iostream>

namespace arm_control {

ArmControlNode::ArmControlNode()
    : Node("arm_control_node"), current_state_(ArmState::IDLE),
      loop_rate_(5), // 状态机频率
      target_detected_(false), target_x_offset_(0.0f), target_y_offset_(0.0f),
      target_class_id_(0), serial_data_(9, 0.0f), idle_timer_(nullptr) {
  // 初始化机械爪为打开状态
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    gripper_state_ = 0.0f;
  }
  
  // 参数
  this->declare_parameter("loop_rate", loop_rate_);
  loop_rate_ = this->get_parameter("loop_rate").as_double();

  // 创建回调组
  auto moveit_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto joint_state_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto servo_publish_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto target_offset_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto serial_data_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // 定时器
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&ArmControlNode::timerCallback, this));

  servo_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArmControlNode::publishServoAnglesCallback, this),
      servo_publish_callback_group);

  // 订阅
  rclcpp::SubscriptionOptions joint_state_options;
  joint_state_options.callback_group = joint_state_callback_group;
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      std::bind(&ArmControlNode::jointStateCallback, this,
                std::placeholders::_1),
      joint_state_options);

  rclcpp::SubscriptionOptions target_offset_options;
  target_offset_options.callback_group = target_offset_callback_group;
  target_offset_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_offset", 10,
      std::bind(&ArmControlNode::targetOffsetCallback, this,
                std::placeholders::_1),
      target_offset_options);

  rclcpp::SubscriptionOptions serial_data_options;
  serial_data_options.callback_group = serial_data_callback_group;
  data_init_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/data_init", 10,
      std::bind(&ArmControlNode::dataInitCallback, this, std::placeholders::_1),
      serial_data_options);

  // 发布器
  servo_angle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/servo_angle", 10);

  // 节点初始化完成后初始化MoveIt接口
  move_group_init_timer_ =
      this->create_wall_timer(std::chrono::seconds(2), [this]() {
        this->initializeMoveIt();
        move_group_init_timer_->cancel();
      });
}

void ArmControlNode::initializeMoveIt() {
  try {
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "group1");

    move_group_->setEndEffectorLink("end_1");
    move_group_->setPlanningTime(2.0);

    // 在单独线程中初始化机器人状态
    std::thread([this]() {
      try {
        robot_state_ = move_group_->getCurrentState();

        if (robot_state_) {
          joint_model_group_ = robot_state_->getJointModelGroup("group1");

          if (joint_model_group_) {
            RCLCPP_INFO(this->get_logger(), "机器人状态初始化成功");
          } else {
            RCLCPP_ERROR(this->get_logger(), "无法获取关节模型组'group1'");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "无法获取当前机器人状态");
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "初始化机器人状态时发生异常: %s", e.what());
      }
    }).detach();

    RCLCPP_INFO(this->get_logger(), "MoveIt接口初始化成功");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "MoveIt初始化失败: %s", e.what());
  }
}

void ArmControlNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_positions_mutex_);
  latest_joint_positions_.resize(4, 0.0);

  // 创建关节名称到索引的映射
  std::map<std::string, size_t> joint_name_to_index;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    joint_name_to_index[msg->name[i]] = i;
  }

  // 按正确顺序排列关节
  const std::vector<std::string> ordered_joint_names = {"joint1", "joint2",
                                                        "joint3", "joint4"};

  for (size_t i = 0; i < ordered_joint_names.size() && i < 4; ++i) {
    const auto &joint_name = ordered_joint_names[i];
    auto it = joint_name_to_index.find(joint_name);

    if (it != joint_name_to_index.end() && it->second < msg->position.size()) {
      latest_joint_positions_[i] = msg->position[it->second];
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "在接收到的关节状态中未找到关节'%s'", joint_name.c_str());
    }
  }

  // 使用FK计算末端执行器位姿
  if (robot_state_ && joint_model_group_ &&
      latest_joint_positions_.size() >= 4) {
    try {
      robot_state_->setJointGroupPositions(joint_model_group_,
                                         latest_joint_positions_);
      robot_state_->update();

      const Eigen::Isometry3d &end_effector_state =
          robot_state_->getGlobalLinkTransform("end_1");

      // 更新当前位姿
      std::lock_guard<std::mutex> pose_lock(current_pose_mutex_);

      current_pose_.position.x = end_effector_state.translation().x();
      current_pose_.position.y = end_effector_state.translation().y();
      current_pose_.position.z = end_effector_state.translation().z();

      Eigen::Quaterniond q(end_effector_state.rotation());
      current_pose_.orientation.x = q.x();
      current_pose_.orientation.y = q.y();
      current_pose_.orientation.z = q.z();
      current_pose_.orientation.w = q.w();
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "FK计算失败: %s", e.what());
    }
  } else if (!robot_state_ || !joint_model_group_) {
    static bool reported = false;
    if (!reported) {
      RCLCPP_ERROR(this->get_logger(), "FK失败: 机器人状态未初始化");
      reported = true;
    }
  }
}

void ArmControlNode::dataInitCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() == 9) {
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    for (size_t i = 0; i < 9; ++i) {
      serial_data_[i] = msg->data[i];
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "接收到的数据大小不匹配: %zu, 预期9个",
              msg->data.size());
  }
}

void ArmControlNode::publishServoAnglesCallback() {
  std_msgs::msg::Float32MultiArray servo_angles;

  {
    std::lock_guard<std::mutex> lock(joint_positions_mutex_);
    if (latest_joint_positions_.empty()) {
      return;
    }

    servo_angles.data.resize(latest_joint_positions_.size() + 1);
    for (size_t i = 0; i < latest_joint_positions_.size(); ++i) {
      servo_angles.data[i] = static_cast<float>(latest_joint_positions_[i]);
    }
    
    float current_gripper_state;
    {
      std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
      current_gripper_state = gripper_state_;
    }
    
    servo_angles.data[latest_joint_positions_.size()] = current_gripper_state;
  }

  servo_angle_pub_->publish(servo_angles);
}

//状态机回调
void ArmControlNode::timerCallback() {
  try {
    static int counter = 0;
    counter++;

    // 每10次迭代记录一次日志
    if (counter % 10 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "状态机迭代: %d", counter);
    }

    // 状态机
    switch (current_state_) {
    case ArmState::IDLE:
      handleIdleState();
      break;
    case ArmState::LEFT_GROUND_GRABBING:
      handleLeftGroundGrabbing();
      break;
    case ArmState::RIGHT_GROUND_GRABBING:
      handleRightGroundGrabbing();
      break;
    case ArmState::HARVESTING:
      handleHarvesting();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "状态机中的未知状态");
      transitionToIdle();
      break;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "定时器回调中的异常: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "定时器回调中的未知异常");
  }
}

//待机状态执行函数
void ArmControlNode::handleIdleState() {
  if (!idle_timer_ || !idle_timer_->is_steady()) {
    idle_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(this->get_logger(), "正在转换到左侧地面抓取状态");
      this->transitionToLeftGroundGrabbing();
      idle_timer_->cancel();
    });
  }
}

//左侧地面抓取状态执行函数
void ArmControlNode::handleLeftGroundGrabbing() {
  float current_x_offset = 0.0f;
  float current_y_offset = 0.0f;

  {
    std::lock_guard<std::mutex> lock(target_offset_mutex_);
    current_x_offset = target_x_offset_;
    current_y_offset = target_y_offset_;
  }

  int count = 0;
  while (current_x_offset == -10000 && current_y_offset == -10000) {
    RCLCPP_INFO(this->get_logger(), "未检测到目标");
    count++;
    if (count > 10) {
      transitionToIdle();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 步骤1: 基于x偏移调整joint1
  RCLCPP_INFO(this->get_logger(), "正在调整joint1...");
  while (1) {
    {
      std::lock_guard<std::mutex> lock(target_offset_mutex_);
      current_x_offset = target_x_offset_;
    }

    // 如果偏移足够小，退出循环
    if (abs(current_x_offset) < 15) {
      RCLCPP_INFO(this->get_logger(), "X轴调整完成");
      break;
    }

    // 计算调整量
    float adjust_joint1_angle = current_x_offset * 0.002;

    // 获取当前关节值
    std::vector<double> joint_values;
    {
      std::lock_guard<std::mutex> lock(joint_positions_mutex_);
      joint_values = latest_joint_positions_;
    }

    if (joint_values.size() >= 1) {
      // 更新joint1角度
      joint_values[0] += adjust_joint1_angle;

      // 规划并执行
      move_group_->setJointValueTarget(joint_values);
      moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
      bool joint_success = static_cast<bool>(move_group_->plan(joint_plan));

      if (joint_success) {
        move_group_->execute(joint_plan);
        
        // 判断真实机械臂状态是否运动到仿真机械臂位置
        if (!waitForJointPositionConvergence(0.05, 3000)) {
          RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到Joint1调整位置");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "机械臂已达到Joint1调整位置");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Joint1调整规划失败");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 步骤2: 基于y偏移调整末端执行器位置
  RCLCPP_INFO(this->get_logger(), "正在调整末端执行器位置...");
  while (1) {
    {
      std::lock_guard<std::mutex> lock(target_offset_mutex_);
      current_y_offset = target_y_offset_;
    }

    // 如果偏移足够小，退出循环
    if (abs(current_y_offset) < 15) {
      RCLCPP_INFO(this->get_logger(), "Y轴调整完成");
      break;
    }

    // 计算调整量
    float adjust_eef_yposition = -current_y_offset * 0.0002;

    // 获取当前位姿
    geometry_msgs::msg::Pose current_pose;
    {
      std::lock_guard<std::mutex> lock(current_pose_mutex_);
      current_pose = current_pose_;
    }

    // 创建旋转矩阵
    Eigen::Quaterniond rotation(
        current_pose.orientation.w, current_pose.orientation.x,
        current_pose.orientation.y, current_pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

    // 计算末端执行器坐标系中的位移
    Eigen::Vector3d y_axis_displacement(0.0, adjust_eef_yposition, 0.0);
    Eigen::Vector3d world_displacement = rotation_matrix * y_axis_displacement;

    // 计算目标位姿
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += world_displacement.x();
    target_pose.position.y += world_displacement.y();
    target_pose.position.z += world_displacement.z();

    // 生成路点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int num_waypoints = 5;
    for (int i = 1; i <= num_waypoints; i++) {
      geometry_msgs::msg::Pose waypoint = current_pose;
      double fraction = static_cast<double>(i) / num_waypoints;

      // 插值位置
      waypoint.position.x =
          current_pose.position.x +
          fraction * (target_pose.position.x - current_pose.position.x);
      waypoint.position.y =
          current_pose.position.y +
          fraction * (target_pose.position.y - current_pose.position.y);
      waypoint.position.z =
          current_pose.position.z +
          fraction * (target_pose.position.z - current_pose.position.z);

      waypoint.orientation = current_pose.orientation;
      waypoints.push_back(waypoint);
    }

    // 规划笛卡尔路径
    double eef_step = 0.01;
    double jump_threshold = 0.0;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "笛卡尔路径完成度: %.2f%%", fraction * 100.0);

    // 创建规划并执行（如果成功）
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    bool success = (fraction > 0.5);

    if (success) {
      move_group_->execute(plan);
      move_group_->stop();
      
      // 判断真实机械臂状态是否运动到仿真机械臂位置
      if (!waitForJointPositionConvergence(0.05, 3000)) {
        RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到抓取位置");
      } else {
        RCLCPP_INFO(this->get_logger(), "机械臂已达到抓取位置");
      }

      // 关闭机械爪抓取目标
      {
        std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
        gripper_state_ = 1.0f;
      }
      
      RCLCPP_INFO(this->get_logger(), "已关闭机械爪抓取目标");
      transitionToHarvesting();
    } else {
      RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划失败");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

//右侧地面抓取状态执行函数
void ArmControlNode::handleRightGroundGrabbing() {
  RCLCPP_DEBUG(this->get_logger(), "正处于右侧地面抓取状态");
  transitionToHarvesting();
}

//收获状态执行函数
void ArmControlNode::handleHarvesting() {
  RCLCPP_DEBUG(this->get_logger(), "正处于收获状态");

  //第一步，调整机械臂pose为预设定的harvest_pose

  //第二步，打开机械爪释放水果
  {
    std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
    gripper_state_ = 0.0f;
  }
  RCLCPP_INFO(this->get_logger(), "已打开机械爪释放水果");

  //第三步，机械臂回到初始位置
  transitionToIdle();  
}

//转换到待机状态
void ArmControlNode::transitionToIdle() {
  RCLCPP_INFO(this->get_logger(), "正在转换到空闲状态");
  current_state_ = ArmState::IDLE;

  try {
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt接口未初始化");
      return;
    }

    move_group_->setNamedTarget("mid_pose");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "规划到mid_pose失败");
      return;
    }

    move_group_->execute(plan);
    
    // 判断真实机械臂状态是否运动到仿真机械臂位置
    if (!waitForJointPositionConvergence(0.05, 5000)) {
      RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到idle位置");
    } else {
      RCLCPP_INFO(this->get_logger(), "机械臂已到达idle位置");
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "转换到空闲状态时异常: %s", e.what());
  }
}

//转换到左侧地面抓取状态
void ArmControlNode::transitionToLeftGroundGrabbing() {
  current_state_ = ArmState::LEFT_GROUND_GRABBING;
  
  try {
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt接口未初始化");
      return;
    }

    move_group_->setNamedTarget("left_pose");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "规划到left_pose失败");
      return;
    }

    move_group_->execute(plan);

    //判断真实机械臂状态是否运动到仿真机械臂位置
    if (!waitForJointPositionConvergence(0.05, 5000)) {
      RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到目标位置");
    } else {
      RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置");
    }
    

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "转换到左侧抓取时异常: %s", e.what());
    transitionToIdle();
  }
}

void ArmControlNode::transitionToRightGroundGrabbing() {
  current_state_ = ArmState::RIGHT_GROUND_GRABBING;
}

void ArmControlNode::transitionToHarvesting() {
  RCLCPP_INFO(this->get_logger(), "正在转换到收获状态");
  current_state_ = ArmState::HARVESTING;
}

void ArmControlNode::targetOffsetCallback(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_offset_mutex_);
  target_x_offset_ = msg->x;
  target_y_offset_ = msg->y;
  target_class_id_ = static_cast<int>(msg->z);
  target_detected_ = true;
}

bool ArmControlNode::waitForJointPositionConvergence(double max_error, int timeout_ms) {
  auto start_time = std::chrono::steady_clock::now();
  
  while (true) {
    // 检查是否超时
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    if (elapsed.count() > timeout_ms) {
      RCLCPP_WARN(this->get_logger(), "等待关节位置收敛超时: %d ms", timeout_ms);
      return false;
    }
    
    // 获取当前关节位置和串口数据
    std::vector<double> current_joint_positions;
    std::vector<float> current_serial_data;
    
    {
      std::lock_guard<std::mutex> joint_lock(joint_positions_mutex_);
      if (latest_joint_positions_.empty() || latest_joint_positions_.size() < 4) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      current_joint_positions = latest_joint_positions_;
    }
    
    {
      std::lock_guard<std::mutex> serial_lock(serial_data_mutex_);
      if (serial_data_.empty() || serial_data_.size() < 4) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      current_serial_data = serial_data_;
    }
    
    // 检查两者差异是否在允许范围内
    bool converged = true;
    for (size_t i = 0; i < 4 && i < current_joint_positions.size(); ++i) {
      double diff = std::abs(current_joint_positions[i] - static_cast<double>(current_serial_data[i]));
      if (diff > max_error) {
        converged = false;
        RCLCPP_DEBUG(this->get_logger(), "关节 %zu 未收敛: 理想值 %.4f, 实际值 %.4f, 差值 %.4f", 
                   i, current_joint_positions[i], current_serial_data[i], diff);
        break;
      }
    }
    
    if (converged) {
      RCLCPP_INFO(this->get_logger(), "所有关节位置已收敛在误差范围内: %.4f", max_error);
      return true;
    }
    
    // 短暂休眠避免CPU占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace arm_control