#include "arm_control/arm_control.hpp"
#include <iostream>

namespace arm_control {

ArmControlNode::ArmControlNode()
    : Node("arm_control_node"), target_detected_(false), target_x_offset_(0.0f),
      target_y_offset_(0.0f), target_class_id_(0), serial_data_(9, 0.0f),
      servo_publish_thread_running_(false) {
  // 初始化机械爪为打开状态
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    gripper_state_ = 0.0f;
  }

  // 创建回调组
  auto joint_state_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto target_offset_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto serial_data_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto action_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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

  // 添加抓取消息发布器
  grab_msg_pub_ =
      this->create_publisher<std_msgs::msg::Int8MultiArray>("/grab_msg", 10);

  // 添加动作服务器
  grab_action_server_ = rclcpp_action::create_server<GrabAction>(
      this, "/grab",
      std::bind(&ArmControlNode::handleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ArmControlNode::handleCancel, this, std::placeholders::_1),
      std::bind(&ArmControlNode::handleAccepted, this, std::placeholders::_1));

  // 启动伺服角度发布线程
  servo_publish_thread_running_ = true;
  servo_publish_thread_ =
      std::thread(&ArmControlNode::servoPublishThreadFunc, this);

  // 节点初始化完成后初始化MoveIt接口
  move_group_init_timer_ =
      this->create_wall_timer(std::chrono::seconds(2), [this]() {
        this->initializeMoveIt();
        move_group_init_timer_->cancel();
      });
}

ArmControlNode::~ArmControlNode() {
  // 停止线程
  servo_publish_thread_running_ = false;
  if (servo_publish_thread_.joinable()) {
    servo_publish_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "Servo发布线程已安全终止");
}

// Action Server Callbacks
rclcpp_action::GoalResponse
ArmControlNode::handleGoal(const rclcpp_action::GoalUUID &uuid,
                           std::shared_ptr<const GrabAction::Goal> goal) {
  // 构建水果种类数组字符串
  std::string types_str = "[";
  for (size_t i = 0; i < goal->type_array.size(); ++i) {
    types_str += std::to_string(goal->type_array[i]);
    if (i < goal->type_array.size() - 1) {
      types_str += ", ";
    }
  }
  types_str += "]";

  RCLCPP_INFO(this->get_logger(),
              "收到抓取请求: 方向=%d, 水果种类数组=%s, 位置=%d(0:地面,1:树上)",
              goal->direction, types_str.c_str(), goal->position);

  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControlNode::handleCancel(
    const std::shared_ptr<GoalHandleGrab> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "收到取消抓取请求");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControlNode::handleAccepted(
    const std::shared_ptr<GoalHandleGrab> goal_handle) {
  // 在新线程中执行抓取操作
  std::thread{std::bind(&ArmControlNode::executeGrabAction, this, goal_handle)}
      .detach();
}

// 执行抓取动作
void ArmControlNode::executeGrabAction(
    const std::shared_ptr<GoalHandleGrab> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "开始执行抓取动作");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GrabAction::Feedback>();
  auto result = std::make_shared<GrabAction::Result>();

  // 发布抓取类型消息
  auto grab_msg = std::make_shared<std_msgs::msg::Int8MultiArray>();

  // 使用完整的水果类型数组
  grab_msg->data.resize(goal->type_array.size());
  for (size_t i = 0; i < goal->type_array.size(); ++i) {
    grab_msg->data[i] = goal->type_array[i];
  }

  grab_msg_pub_->publish(*grab_msg);

  feedback->completion_percentage = 10.0;
  goal_handle->publish_feedback(feedback);

  bool success = true;

  // 根据抓取方向和位置执行不同操作
  if (goal->direction == 0) {  // 左侧
    if (goal->position == 0) { // 地面
      RCLCPP_INFO(this->get_logger(), "执行左侧地面抓取序列");

      feedback->completion_percentage = 20.0;
      goal_handle->publish_feedback(feedback);

      // 执行左侧地面抓取序列
      if (!transitionToLeftGroundGrabbing()) {
        RCLCPP_ERROR(this->get_logger(), "转换到左侧地面抓取姿态失败");
        result->message = "转换到左侧地面抓取姿态失败";
        // success = false;
      }

      if (success) {
        feedback->completion_percentage = 40.0;
        goal_handle->publish_feedback(feedback);

        // 执行抓取操作
        if (!handleLeftGroundGrabbing()) {
          RCLCPP_ERROR(this->get_logger(), "左侧地面抓取操作失败");
          result->message = "左侧地面抓取操作失败";
          // success = false;
        }
      }

      if (success) {
        feedback->completion_percentage = 60.0;
        goal_handle->publish_feedback(feedback);

        // 转换到收获位置
        if (!transitionToHarvesting()) {
          RCLCPP_ERROR(this->get_logger(), "转换到收获位置失败");
          result->message = "转换到收获位置失败";
          // success = false;
        }
      }
      if (success) {
        feedback->completion_percentage = 80.0;
        goal_handle->publish_feedback(feedback);

        // 执行收获操作
        if (!handleHarvesting()) {
          RCLCPP_ERROR(this->get_logger(), "收获操作失败");
          result->message = "收获操作失败";
          // success = false;
        } else {
          feedback->completion_percentage = 90.0;
          goal_handle->publish_feedback(feedback);
        }
      }
    }
  } else {  // 右侧
    if (goal->position == 0) { // 地面
      RCLCPP_INFO(this->get_logger(), "执行右侧地面抓取序列");
      // TODO: 实现右侧地面抓取逻辑
    }
  }

  // 设置最终结果
  if (success) {
    feedback->completion_percentage = 100.0;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    if (result->message.empty()) {
      result->message = "抓取操作成功完成";
    }
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "抓取动作成功完成");
  } else {
    result->success = false;
    if (result->message.empty()) {
      result->message = "抓取操作失败";
    }
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "抓取动作失败: %s",
                 result->message.c_str());
  }

  transitionToIdle();
}

// 伺服角度发布线程
void ArmControlNode::servoPublishThreadFunc() {
  RCLCPP_INFO(this->get_logger(), "伺服角度发布线程已启动");

  while (servo_publish_thread_running_ && rclcpp::ok()) {
    // 创建伺服角度消息
    std_msgs::msg::Float32MultiArray servo_angles;

    {
      std::lock_guard<std::mutex> lock(joint_positions_mutex_);
      if (latest_joint_positions_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
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

    // 发布消息
    servo_angle_pub_->publish(servo_angles);

    // 按固定频率发布
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "伺服角度发布线程结束");
}

// 初始化MoveIt接口
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
        RCLCPP_ERROR(this->get_logger(), "初始化机器人状态时发生异常: %s",
                     e.what());
      }
    }).detach();

    RCLCPP_INFO(this->get_logger(), "MoveIt接口初始化成功");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "MoveIt初始化失败: %s", e.what());
  }
}

// 关节状态更新
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
                           "在接收到的关节状态中未找到关节'%s'",
                           joint_name.c_str());
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

// 更新下位机数据
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

// 左侧地面抓取执行函数
bool ArmControlNode::handleLeftGroundGrabbing() {
  RCLCPP_INFO(this->get_logger(), "执行左侧地面抓取操作");

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
      return false;
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
    if (abs(current_x_offset) < 30) {
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
          RCLCPP_WARN(this->get_logger(),
                      "机械臂未能在指定时间内达到Joint1调整位置");
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

    // 使用新的笛卡尔运动函数
    bool success =
        moveEndEffectorCartesian(0.0, adjust_eef_yposition, 0.0, 3000);

    if (success) {
      // 判断真实机械臂状态是否运动到仿真机械臂位置
      if (!waitForJointPositionConvergence(0.05, 3000)) {
        RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到目标位置");
      } else {
        RCLCPP_INFO(this->get_logger(), "机械臂已达到目标位置");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // 运动到抓取姿态
  float adjust_eff_zposition = 0.21;
  float adjust_eff_yposition = 0.03;
  float adjust_eff_xposition = 0.0;
  bool grab_success = moveEndEffectorCartesian(
      adjust_eff_xposition, adjust_eff_yposition, adjust_eff_zposition, 3000);

  // if (grab_success) {
  //   // 判断真实机械臂状态是否运动到仿真机械臂位置
  //   if (!waitForJointPositionConvergence(0.05, 3000)) {
  //     RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到抓取位置");
  //     return false;
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "机械臂已达到抓取位置");

  //     // 关闭机械爪抓取目标
  //     {
  //       std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
  //       gripper_state_ = 1.0f;
  //     }

  //     RCLCPP_INFO(this->get_logger(), "已关闭机械爪抓取目标");
  //     return true;
  //   }
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "移动到抓取位置失败");
  //   return false;
  // }
      // 关闭机械爪抓取目标
      {
        std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
        gripper_state_ = 1.0f;
      }
}

// 右侧地面抓取执行函数
bool ArmControlNode::handleRightGroundGrabbing() {
  RCLCPP_INFO(this->get_logger(), "执行右侧地面抓取操作");
  // TODO: 实现右侧地面抓取逻辑
}

// 收获执行函数
bool ArmControlNode::handleHarvesting() {
  RCLCPP_DEBUG(this->get_logger(), "正处于收获状态");

  // 打开机械爪释放水果
  {
    std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
    gripper_state_ = 0.0f;
  }
  RCLCPP_INFO(this->get_logger(), "已打开机械爪释放水果");

  // TODO: 等待真实夹爪状态为张开状态
  std::this_thread::sleep_for(
      std::chrono::milliseconds(500)); // 简单等待夹爪打开
  return true;
}

// 转换到待机姿态
bool ArmControlNode::transitionToIdle() {
  RCLCPP_INFO(this->get_logger(), "正在转换到空闲状态");

  move_group_->setNamedTarget("mid_pose");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "规划到mid_pose失败");
    return false; // 规划失败，返回失败
  }

  move_group_->execute(plan);

  // 判断真实机械臂状态是否运动到仿真机械臂位置
  if (!waitForJointPositionConvergence(0.05, 5000)) {
    RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到idle位置");
    return false; // 位置未收敛，返回失败
  } else {
    RCLCPP_INFO(this->get_logger(), "机械臂已到达idle位置");
    return true; // 成功到达idle位置
  }
}

// 转换到左侧地面抓取姿态
bool ArmControlNode::transitionToLeftGroundGrabbing() {
  RCLCPP_INFO(this->get_logger(), "正在转换到左侧地面抓取状态");

  move_group_->setNamedTarget("left_pose");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "规划到left_pose失败");
    return false; // 规划失败，返回失败
  }

  move_group_->execute(plan);

  // 判断真实机械臂状态是否运动到仿真机械臂位置
  if (!waitForJointPositionConvergence(0.05, 5000)) {
    RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到目标位置");
    return false; // 位置未收敛，返回失败
  } else {
    RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置");
    return true; // 成功到达目标位置
  }
}

// 转换到右侧地面抓取姿态
bool ArmControlNode::transitionToRightGroundGrabbing() {
  RCLCPP_INFO(this->get_logger(), "正在转换到右侧地面抓取状态");

  // 可以添加右侧抓取的具体实现
  move_group_->setNamedTarget("right_pose"); // 假设存在"right_pose"命名位姿
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "规划到right_pose失败");
    return false; // 规划失败，返回失败
  }

  move_group_->execute(plan);

  // 判断真实机械臂状态是否运动到仿真机械臂位置
  if (!waitForJointPositionConvergence(0.05, 5000)) {
    RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到目标位置");
    return false; // 位置未收敛，返回失败
  } else {
    RCLCPP_INFO(this->get_logger(), "机械臂已到达右侧抓取位置");
    return true; // 成功到达目标位置
  }
}

// 使用笛卡尔运动函数移动末端执行器
bool ArmControlNode::moveEndEffectorCartesian(double x_displacement,
                                              double y_displacement,
                                              double z_displacement,
                                              int timeout_ms) {
  // Get current pose
  geometry_msgs::msg::Pose current_pose;
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose = current_pose_;
  }

  // Create rotation matrix
  Eigen::Quaterniond rotation(
      current_pose.orientation.w, current_pose.orientation.x,
      current_pose.orientation.y, current_pose.orientation.z);
  Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

  // Calculate end-effector displacement in end-effector frame
  Eigen::Vector3d eef_displacement(x_displacement, y_displacement,
                                   z_displacement);

  // Transform to world displacement
  Eigen::Vector3d world_displacement = rotation_matrix * eef_displacement;

  // Calculate target pose
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x += world_displacement.x();
  target_pose.position.y += world_displacement.y();
  target_pose.position.z += world_displacement.z();

  // Generate waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  int num_waypoints = 5;
  for (int i = 1; i <= num_waypoints; i++) {
    geometry_msgs::msg::Pose waypoint = current_pose;
    double fraction = static_cast<double>(i) / num_waypoints;

    // Interpolate position
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

  // Plan cartesian path
  double eef_step = 0.01;
  double jump_threshold = 0.0;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(this->get_logger(), "笛卡尔路径完成度: %.2f%%", fraction * 100.0);

  // Create plan and execute (if successful)
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  bool success = (fraction > 0.5);

  if (success) {
    move_group_->execute(plan);
    move_group_->stop();
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划失败");
    return false;
  }
}

bool ArmControlNode::transitionToHarvesting() {
  RCLCPP_INFO(this->get_logger(), "正在转换到收获状态");

  // 运动到预设的harvest_pose1
  move_group_->setNamedTarget("harvest_pose1");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "规划到harvest_pose1失败");
    return false; // 规划失败，返回失败
  }

  move_group_->execute(plan);

  // 判断真实机械臂状态是否运动到仿真机械臂位置
  if (!waitForJointPositionConvergence(0.05, 5000)) {
    RCLCPP_WARN(this->get_logger(), "机械臂未能在指定时间内达到目标位置");
    return false; // 位置未收敛，返回失败
  } else {
    RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置");
    return true; // 成功到达目标位置
  }
}

void ArmControlNode::targetOffsetCallback(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_offset_mutex_);
  target_x_offset_ = msg->x;
  target_y_offset_ = msg->y;
  target_class_id_ = static_cast<int>(msg->z);
  target_detected_ = true;
}

bool ArmControlNode::waitForJointPositionConvergence(double max_error,
                                                     int timeout_ms) {
  auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // 检查是否超时
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - start_time);
    if (elapsed.count() > timeout_ms) {
      RCLCPP_WARN(this->get_logger(), "等待关节位置收敛超时: %d ms",
                  timeout_ms);
      return false;
    }

    // 获取当前关节位置和串口数据
    std::vector<double> current_joint_positions;
    std::vector<float> current_serial_data;

    {
      std::lock_guard<std::mutex> joint_lock(joint_positions_mutex_);
      if (latest_joint_positions_.empty() ||
          latest_joint_positions_.size() < 4) {
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
      double diff = std::abs(current_joint_positions[i] -
                             static_cast<double>(current_serial_data[i]));
      if (diff > max_error) {
        converged = false;
        RCLCPP_DEBUG(this->get_logger(),
                     "关节 %zu 未收敛: 理想值 %.4f, 实际值 %.4f, 差值 %.4f", i,
                     current_joint_positions[i], current_serial_data[i], diff);
        break;
      }
    }

    if (converged) {
      RCLCPP_INFO(this->get_logger(), "所有关节位置已收敛在误差范围内: %.4f",
                  max_error);
      return true;
    }

    // 短暂休眠避免CPU占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace arm_control