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
  
  // 声明参数
  this->declare_parameter("loop_rate", loop_rate_);

  // 获取参数
  loop_rate_ = this->get_parameter("loop_rate").as_double();

  // 初始化MoveIt
  // 注意：使用回调组在单独的线程中处理MoveIt初始化
  // 因为它可能需要一些时间并阻塞节点初始化
  auto moveit_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 创建一个并行回调组，用于关节状态更新
  auto joint_state_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // 创建并行回调组用于伺服角度发布
  auto servo_publish_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // 创建并行回调组用于目标偏移订阅
  auto target_offset_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // 创建并行回调组用于串口数据订阅
  auto serial_data_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // 创建状态机循环的定时器
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz = 每100ms
                              std::bind(&ArmControlNode::timerCallback, this));

  // 创建100ms的伺服角度发布定时器，使用并行回调组
  servo_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArmControlNode::publishServoAnglesCallback, this),
      servo_publish_callback_group);

  // 创建关节状态订阅者，使用并行回调组
  rclcpp::SubscriptionOptions joint_state_options;
  joint_state_options.callback_group = joint_state_callback_group;
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      std::bind(&ArmControlNode::jointStateCallback, this,
                std::placeholders::_1),
      joint_state_options);

  // 创建目标偏移订阅者，使用并行回调组
  rclcpp::SubscriptionOptions target_offset_options;
  target_offset_options.callback_group = target_offset_callback_group;
  target_offset_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_offset", 10,
      std::bind(&ArmControlNode::targetOffsetCallback, this,
                std::placeholders::_1),
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
  move_group_init_timer_ =
      this->create_wall_timer(std::chrono::seconds(2), [this]() {
        this->initializeMoveIt();
        // 初始化完成后取消定时器
        move_group_init_timer_->cancel();
      });
}

// 初始化MoveIt接口的方法
void ArmControlNode::initializeMoveIt() {
  try {
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "group1");

    move_group_->setEndEffectorLink("end_1");

    // 设置规划时长为10秒
    move_group_->setPlanningTime(2.0);

    // 创建异步任务初始化机器人状态
    std::thread([this]() {
      try {
        // 获取当前机器人状态
        robot_state_ = move_group_->getCurrentState();

        if (robot_state_) {
          // 获取关节模型组
          joint_model_group_ = robot_state_->getJointModelGroup("group1");

          if (joint_model_group_) {
            RCLCPP_INFO(
                this->get_logger(),
                "异步初始化机器人状态成功，关节模型组已准备好用于FK计算");
          } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "异步初始化机器人状态失败：无法获取关节模型组 'group1'");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "异步初始化机器人状态失败：无法获取当前机器人状态");
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "异步初始化机器人状态时发生异常: %s",
                     e.what());
      }
    }).detach(); // 分离线程，让它在后台运行

    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized successfully "
                                    "with planning group 'group1'");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize MoveIt interface: %s", e.what());
  }
}

void ArmControlNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 更新最新的关节位置
  std::lock_guard<std::mutex> lock(joint_positions_mutex_);

  // 确保数据结构大小正确（4个关节）
  latest_joint_positions_.resize(4, 0.0);

  // 创建名称到索引的映射
  std::map<std::string, size_t> joint_name_to_index;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    joint_name_to_index[msg->name[i]] = i;
  }

  // 按照指定顺序（joint1, joint2, joint3, joint4）重新排序关节位置
  const std::vector<std::string> ordered_joint_names = {"joint1", "joint2",
                                                        "joint3", "joint4"};

  for (size_t i = 0; i < ordered_joint_names.size() && i < 4; ++i) {
    const auto &joint_name = ordered_joint_names[i];
    auto it = joint_name_to_index.find(joint_name);

    if (it != joint_name_to_index.end() && it->second < msg->position.size()) {
      latest_joint_positions_[i] = msg->position[it->second];
      RCLCPP_DEBUG(this->get_logger(),
                   "Mapped %s to position %zu (value: %.4f)",
                   joint_name.c_str(), i, latest_joint_positions_[i]);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Joint '%s' not found in received joint states",
                           joint_name.c_str());
    }
  }

  // 使用前向运动学计算末端执行器姿态，直接使用预先初始化的robot_state_和joint_model_group_
  if (robot_state_ && joint_model_group_ &&
      latest_joint_positions_.size() >= 4) {
    try {
      // 直接设置关节角度到已有的机器人状态
      robot_state_->setJointGroupPositions(joint_model_group_,
                                           latest_joint_positions_);

      // 更新机器人状态中的变换
      robot_state_->update();

      // 计算末端执行器位置
      const Eigen::Isometry3d &end_effector_state =
          robot_state_->getGlobalLinkTransform("end_1");

      // 转换为几何消息格式并更新当前位姿
      {
        std::lock_guard<std::mutex> pose_lock(current_pose_mutex_);

        // 位置
        current_pose_.position.x = end_effector_state.translation().x();
        current_pose_.position.y = end_effector_state.translation().y();
        current_pose_.position.z = end_effector_state.translation().z();

        // 方向（将旋转矩阵转换为四元数）
        Eigen::Quaterniond q(end_effector_state.rotation());
        current_pose_.orientation.x = q.x();
        current_pose_.orientation.y = q.y();
        current_pose_.orientation.z = q.z();
        current_pose_.orientation.w = q.w();

        RCLCPP_DEBUG(this->get_logger(),
                     "FK计算的末端位置: x=%.4f, y=%.4f, z=%.4f",
                     current_pose_.position.x, current_pose_.position.y,
                     current_pose_.position.z);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "前向运动学计算失败: %s", e.what());
    }
  } else if (!robot_state_ || !joint_model_group_) {
    // 只在初始化后第一次报告问题
    static bool reported = false;
    if (!reported) {
      RCLCPP_ERROR(
          this->get_logger(),
          "FK计算失败：机器人状态或关节模型组未初始化，请检查MoveIt初始化");
      reported = true;
    }
  }
}

// Data init callback 处理从串口接收到的数据
void ArmControlNode::dataInitCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // 检查消息中的数据是否符合预期
  if (msg->data.size() == 9) {
    // 线程安全地更新数据
    std::lock_guard<std::mutex> lock(serial_data_mutex_);

    // 复制数据到成员变量
    for (size_t i = 0; i < 9; ++i) {
      serial_data_[i] = msg->data[i];
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "接收到串口数据: "
                 "servo1=%.2f, servo2=%.2f, servo3=%.2f, servo4=%.2f, "
                 "encoder1=%.2f, encoder2=%.2f, encoder3=%.2f, encoder4=%.2f, "
                 "imu_yaw=%.2f",
                 serial_data_[0], serial_data_[1], serial_data_[2],
                 serial_data_[3], serial_data_[4], serial_data_[5],
                 serial_data_[6], serial_data_[7], serial_data_[8]);
  } else {
    RCLCPP_WARN(this->get_logger(), "接收到的数据大小不符合预期: %zu, 预期 9",
                msg->data.size());
  }
}

void ArmControlNode::publishServoAnglesCallback() {
  // 创建Float32MultiArray消息并发布最新的关节角度数据
  std_msgs::msg::Float32MultiArray servo_angles;

  {
    // 线程安全地访问最新的关节位置
    std::lock_guard<std::mutex> lock(joint_positions_mutex_);

    // 如果还没有接收到关节数据，不发布
    if (latest_joint_positions_.empty()) {
      return;
    }

    // 将关节角度数据复制到Float32MultiArray消息，并增加一个元素用于机械爪状态
    servo_angles.data.resize(latest_joint_positions_.size() + 1);
    for (size_t i = 0; i < latest_joint_positions_.size(); ++i) {
      servo_angles.data[i] = static_cast<float>(latest_joint_positions_[i]);
    }
    
    // 线程安全地读取机械爪状态
    float current_gripper_state;
    {
      std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
      current_gripper_state = gripper_state_;
    }
    
    // 添加机械爪状态作为最后一个元素
    servo_angles.data[latest_joint_positions_.size()] = current_gripper_state;
  }

  // 发布伺服角度消息
  servo_angle_pub_->publish(servo_angles);

  RCLCPP_DEBUG(this->get_logger(), "Published %zu joint angles and gripper state to /servo_angle",
               servo_angles.data.size());
}

void ArmControlNode::timerCallback() {
  try {
    static int counter = 0;
    counter++;

    // 每10次才打印一次，避免输出过多
    if (counter % 10 == 0) {
      std::cout << "timerCallback executing... count: " << counter << std::endl;
    }

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
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in timerCallback: %s",
                 e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception in timerCallback");
  }
}

void ArmControlNode::handleIdleState() {
  // 创建一次性定时器，等待5秒后切换到左侧地面抓取状态
  if (!idle_timer_ || !idle_timer_->is_steady()) {
    std::cout << "Creating idle timer..." << std::endl;
    idle_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(
          this->get_logger(),
          "5-second wait completed, transitioning to left ground grabbing");
      this->transitionToLeftGroundGrabbing();
      // 一次性定时器，触发后自动取消
      idle_timer_->cancel();
    });
  }
}

void ArmControlNode::handleLeftGroundGrabbing() {
  float current_x_offset = 0.0f;
  float current_y_offset = 0.0f;

  // 获取目标偏移量
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

  // 第一步：调整joint1角度，基于current_x_offset
  RCLCPP_INFO(this->get_logger(), "开始调整joint1角度...");
  while (1) {
    // 线程安全地访问目标偏移变量
    {
      std::lock_guard<std::mutex> lock(target_offset_mutex_);
      current_x_offset = target_x_offset_;
    }

    RCLCPP_INFO(this->get_logger(), "当前X轴偏移: %.2f", current_x_offset);

    // 如果X轴偏移足够小，跳出循环
    if (abs(current_x_offset) < 15) {
      RCLCPP_INFO(this->get_logger(), "X轴偏移调整完成");
      break;
    }

    // 计算joint1角度调整量
    float adjust_joint1_angle = current_x_offset * 0.002;

    // 获取当前关节角度
    std::vector<double> joint_values;
    {
      std::lock_guard<std::mutex> lock(joint_positions_mutex_);
      joint_values = latest_joint_positions_;
    }

    // 确保有足够的关节数据
    if (joint_values.size() >= 1) {
      // 计算joint1的新角度
      joint_values[0] += adjust_joint1_angle;

      // 设置新的关节角度
      move_group_->setJointValueTarget(joint_values);

      // 规划并执行
      moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
      bool joint_success = static_cast<bool>(move_group_->plan(joint_plan));

      if (joint_success) {
        RCLCPP_INFO(this->get_logger(), "执行joint1角度调整...");
        move_group_->execute(joint_plan);
      } else {
        RCLCPP_ERROR(this->get_logger(), "joint1角度调整规划失败");
      }
    }

    // 控制循环频率
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 第二步：调整末端执行器位置，基于current_y_offset
  RCLCPP_INFO(this->get_logger(), "开始调整末端执行器位置...");
  while (1) {
    // 线程安全地访问目标偏移变量
    {
      std::lock_guard<std::mutex> lock(target_offset_mutex_);
      current_y_offset = target_y_offset_;
    }

    RCLCPP_INFO(this->get_logger(), "当前Y轴偏移: %.2f", current_y_offset);

    // 如果Y轴偏移足够小，跳出循环
    if (abs(current_y_offset) < 15) {
      RCLCPP_INFO(this->get_logger(), "Y轴偏移调整完成");
      break;
    }

    // 计算末端执行器Y轴调整量
    float adjust_eef_yposition = -current_y_offset * 0.0002;

    // 获取当前位姿
    geometry_msgs::msg::Pose current_pose;
    {
      std::lock_guard<std::mutex> lock(current_pose_mutex_);
      current_pose = current_pose_;
    }

    // 输出当前位置信息
    RCLCPP_INFO(this->get_logger(),
                "当前末端执行器位置: x=%.4f, y=%.4f, z=%.4f",
                current_pose.position.x, current_pose.position.y,
                current_pose.position.z);

    // 创建表示末端执行器自身坐标系的旋转矩阵
    Eigen::Quaterniond rotation(
        current_pose.orientation.w, current_pose.orientation.x,
        current_pose.orientation.y, current_pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

    // 在末端执行器自身坐标系的Y轴方向上移动adjust_eef_yposition米
    Eigen::Vector3d y_axis_displacement(0.0, adjust_eef_yposition, 0.0);

    // 将位移转换到世界坐标系
    Eigen::Vector3d world_displacement = rotation_matrix * y_axis_displacement;

    // 计算新的目标位置
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += world_displacement.x();
    target_pose.position.y += world_displacement.y();
    target_pose.position.z += world_displacement.z();

    RCLCPP_INFO(
        this->get_logger(), "目标末端执行器位置: x=%.4f, y=%.4f, z=%.4f",
        target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 创建一个笛卡尔路径的路点列表
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 生成中间路点以创建更平滑的路径
    // 当前位置到目标位置的距离分为5个路点
    int num_waypoints = 5;
    for (int i = 1; i <= num_waypoints; i++) {
      geometry_msgs::msg::Pose waypoint = current_pose;
      double fraction = static_cast<double>(i) / num_waypoints;

      // 线性插值当前位置和目标位置之间的路点
      waypoint.position.x =
          current_pose.position.x +
          fraction * (target_pose.position.x - current_pose.position.x);
      waypoint.position.y =
          current_pose.position.y +
          fraction * (target_pose.position.y - current_pose.position.y);
      waypoint.position.z =
          current_pose.position.z +
          fraction * (target_pose.position.z - current_pose.position.z);

      // 保持相同的方向
      waypoint.orientation = current_pose.orientation;

      // 添加路点
      waypoints.push_back(waypoint);
    }

    // 设置笛卡尔路径的参数
    double eef_step = 0.01;      // 末端执行器的最大步长(米)
    double jump_threshold = 0.0; // 禁用跳跃检测

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group_->computeCartesianPath(waypoints,      // 路点
                                          eef_step,       // 末端步长
                                          jump_threshold, // 跳跃阈值
                                          trajectory      // 输出轨迹
        );

    RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划完成度: %.2f%%",
                fraction * 100.0);

    // 创建一个计划对象
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    // 如果路径规划至少50%成功，执行移动
    bool success = (fraction > 0.5);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "执行笛卡尔路径...");

      // 执行计划的轨迹
      move_group_->execute(plan);

      // 等待运动完成
      move_group_->stop(); // 确保前一个运动完全停止

      // 线程安全地关闭夹爪抓取目标
      {
        std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
        gripper_state_ = 1.0f;
      }
      
      RCLCPP_INFO(this->get_logger(), "已关闭夹爪抓取目标");
      
      // 此处可以添加控制夹持器抓取的代码
      RCLCPP_INFO(this->get_logger(), "抓取完成，转换到收获状态");
      
      // 抓取完成后转到收获状态
      transitionToHarvesting();
    } else {
      RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划失败或完成度太低");
      // 尝试使用较小的移动距离
      RCLCPP_INFO(this->get_logger(), "尝试使用较小的移动距离...");
    }

    // 控制循环频率
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void ArmControlNode::handleRightGroundGrabbing() {
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

void ArmControlNode::handleHarvesting() {
  RCLCPP_DEBUG(this->get_logger(), "Handling HARVESTING state");

  /* 实现将包括：
   * 1. 将机械臂移动到收集箱以存放水果
   * 2. 释放夹持器
   * 3. 记录收获的水果
   * 4. 转换到下一个状态
   */

  // 线程安全地打开夹爪释放水果
  {
    std::lock_guard<std::mutex> grip_lock(gripper_state_mutex_);
    gripper_state_ = 0.0f;
  }
  
  RCLCPP_INFO(this->get_logger(), "已打开夹爪释放水果");
  
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
void ArmControlNode::transitionToIdle() {
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
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %.4f rad (%.2f deg)", i + 1,
                  joint_values[i], joint_values[i] * 180.0 / M_PI);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in transitioning to idle: %s",
                 e.what());
  }
}

void ArmControlNode::transitionToLeftGroundGrabbing() {

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
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %.4f rad (%.2f deg)", i + 1,
                  joint_values[i], joint_values[i] * 180.0 / M_PI);
    }

    /*debug
    // 等待舵机实际到达规划位置
    const double angle_tolerance = 0.05; // 角度误差容忍范围（弧度）
    const int max_wait_cycles = 100;     // 最大等待循环次数
    int wait_cycles = 0;

    RCLCPP_INFO(this->get_logger(), "等待舵机到达目标位置...");

    while (wait_cycles < max_wait_cycles) {
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
        RCLCPP_WARN(this->get_logger(),
                    "没有足够的关节数据进行比较，等待下一次数据更新");
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
          RCLCPP_DEBUG(this->get_logger(),
                       "关节 %zu 误差: %.2f 度 (规划:%.2f, 实际:%.2f)", i + 1,
                       error, planned_angle_deg, actual_angle_deg);
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
    */
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in left ground grabbing: %s",
                 e.what());
    transitionToIdle();
  }
}

void ArmControlNode::transitionToRightGroundGrabbing() {
  current_state_ = ArmState::RIGHT_GROUND_GRABBING;
}

void ArmControlNode::transitionToHarvesting() {
  RCLCPP_INFO(this->get_logger(), "Transitioning to HARVESTING state");
  current_state_ = ArmState::HARVESTING;

  // 可以在这里实现额外的转换动作
}

void ArmControlNode::targetOffsetCallback(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  // 线程安全地更新目标偏移数据
  {
    std::lock_guard<std::mutex> lock(target_offset_mutex_);
    target_x_offset_ = msg->x;
    target_y_offset_ = msg->y;
    target_class_id_ = static_cast<int>(msg->z);
    target_detected_ = true;
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Received target offset: x=%.2f, y=%.2f, class_id=%d",
               target_x_offset_, target_y_offset_, target_class_id_);
}

} // namespace arm_control