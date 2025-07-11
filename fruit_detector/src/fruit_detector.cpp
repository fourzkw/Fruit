#include "fruit_detector/fruit_detector.hpp"
#include <memory>

namespace fruit_detector {

FruitDetector::FruitDetector(const rclcpp::NodeOptions &options)
    : Node("fruit_detector", options), first_frame_(true), current_fps_(0.0f) {
  // 声明参数
  this->declare_parameter("image_topic", "/image_raw");
  this->declare_parameter("camera_info_topic", "/camera_info");
  this->declare_parameter("model_path", "");
  this->declare_parameter("conf_threshold", 0.25f);
  this->declare_parameter("iou_threshold", 0.45f);

  // 获取参数
  image_topic_ = this->get_parameter("image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  model_path_ = this->get_parameter("model_path").as_string();
  conf_threshold_ = this->get_parameter("conf_threshold").as_double();
  iou_threshold_ = this->get_parameter("iou_threshold").as_double();

  RCLCPP_INFO(this->get_logger(), "初始化检测器, 模型路径: %s",
              model_path_.c_str());

  // 初始化推理模型
  inference_ = std::make_unique<yolo::Inference>(model_path_, conf_threshold_,
                                                 iou_threshold_);

  // 创建发布者
  detection_publisher_ =
      this->create_publisher<vision_msgs::msg::Detection2DArray>("detections",
                                                                 10);
  image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "detection_image", 10); // 创建推理图像发布器
  target_offset_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/target_offset", 10); // 创建目标偏移发布器

  // 创建订阅者
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&FruitDetector::imageCallback, this, std::placeholders::_1));

  camera_info_subscription_ =
      this->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic_, 10,
          [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            camera_info_ = msg;
          });
          
  // 添加抓取消息订阅
  grab_msg_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "/grab_msg", 10,
      std::bind(&FruitDetector::grabMsgCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "水果检测节点初始化完成");
}

// 实现抓取消息回调函数
void FruitDetector::grabMsgCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
  // 更新目标水果类型数组
  target_fruit_types_.clear();
  target_fruit_types_.assign(msg->data.begin(), msg->data.end());
  
  // 构建水果类型数组字符串用于日志
  std::string types_str = "[";
  for (size_t i = 0; i < target_fruit_types_.size(); ++i) {
    types_str += std::to_string(target_fruit_types_[i]);
    if (i < target_fruit_types_.size() - 1) {
      types_str += ", ";
    }
  }
  types_str += "]";
  
  RCLCPP_INFO(this->get_logger(), "收到目标水果类型数组: %s", types_str.c_str());
}

void FruitDetector::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // 计算帧率
    auto now = std::chrono::steady_clock::now();
    if (!first_frame_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_frame_time_).count();
      if (elapsed > 0) {
        current_fps_ = 1000.0f / static_cast<float>(elapsed);
      }
    } else {
      first_frame_ = false;
    }
    last_frame_time_ = now;
    
    // 转换ROS图像消息为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    // 执行推理
    inference_->RunInference(frame);

    // 获取检测结果
    auto detections = inference_->GetContours();

    if (!detections.empty()) {
      auto target_detection = detections[0];
      float target_x_offset = 640; // 更新为1280的一半
      float target_y_offset = 480; // 更新为960的一半
      int target_class_id = 0;
      bool target_found = false;

      for (auto detection : detections)
      {
        int detected_class_id = (int)detection[0].x;
        
        // 检查当前检测的类别是否在目标水果类型列表中
        bool is_target_class = target_fruit_types_.empty(); // 如果没有指定目标类型，则接受所有类型
        
        for (const auto& target_type : target_fruit_types_) {
          if (detected_class_id == target_type) {
            is_target_class = true;
            break;
          }
        }
        
        // 如果类型不匹配，跳过此检测
        if (!is_target_class) {
          continue;
        }
        
        float center_x = (detection[1].x + detection[2].x) / 2;
        float center_y = (detection[1].y + detection[2].y) / 2;
        float x_offset = 640 - center_x; // 更新为1280的一半
        float y_offset = 480 - center_y; // 更新为960的一半

        // 在图像上绘制中心点，目标类型用红色，其他用黄色
        cv::Scalar circle_color = is_target_class ? 
                                  cv::Scalar(0, 0, 255) :  // 红色表示目标水果
                                  cv::Scalar(0, 255, 255); // 黄色表示非目标水果
        cv::circle(frame, cv::Point(center_x, center_y), 5,
                   circle_color, -1);

        if (is_target_class && 
            (!target_found || 
             (abs(x_offset) + abs(y_offset) < abs(target_x_offset) + abs(target_y_offset)))) {
          target_detection = detection;
          target_x_offset = x_offset;
          target_y_offset = y_offset;
          target_class_id = detected_class_id;
          target_found = true;
        }
      }

      // 绘制测试图像标识
      cv::line(frame, cv::Point(640, 0), cv::Point(640, 960), // 垂直线通过中心点
               cv::Scalar(0, 255, 0), 1);
      cv::line(frame, cv::Point(0, 480), cv::Point(1280, 480), // 水平线通过中心点
               cv::Scalar(0, 255, 0), 1);
      cv::putText(frame, "target_x_offset: " + std::to_string(target_x_offset),
                  cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, // 增大字体
                  cv::Scalar(0, 0, 255), 2);
      cv::putText(frame, "target_y_offset: " + std::to_string(target_y_offset),
                  cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, // 增大字体
                  cv::Scalar(0, 0, 255), 2);
                  
      // 发布目标偏移量
      geometry_msgs::msg::Point offset_msg;
      offset_msg.x = target_x_offset;
      offset_msg.y = target_y_offset;
      offset_msg.z = target_class_id;
      target_offset_publisher_->publish(offset_msg);
    }
    else
    {
      // 发布目标偏移量
      geometry_msgs::msg::Point offset_msg;
      offset_msg.x = -10000;
      offset_msg.y = -10000;
      offset_msg.z = -1;
      target_offset_publisher_->publish(offset_msg);
    }
    
    // 添加FPS显示
    char fps_text[50];
    snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", current_fps_);
    cv::putText(frame, fps_text, cv::Point(20, 150), cv::FONT_HERSHEY_SIMPLEX, 
                1.0, cv::Scalar(0, 255, 0), 2);

    // 创建并发布检测消息
    auto detection_msg = createDetectionMessage(detections, msg->header);
    detection_publisher_->publish(detection_msg);

    // 发布带有检测结果的图像
    cv_bridge::CvImage detection_image;
    detection_image.header = msg->header;
    detection_image.encoding = sensor_msgs::image_encodings::BGR8;
    detection_image.image = frame;

    // 在发布前确认图像有效
    if (!frame.empty()) {
      auto image_msg = detection_image.toImageMsg();
      image_publisher_->publish(*image_msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "检测结果图像为空，无法发布");
    }

  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "CV桥接异常: %s", e.what());
  } catch (std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "处理异常: %s", e.what());
  }
}

vision_msgs::msg::Detection2DArray FruitDetector::createDetectionMessage(
    const std::vector<std::vector<cv::Point2f>> &detections,
    const std_msgs::msg::Header &header) {
  vision_msgs::msg::Detection2DArray detection_array;
  detection_array.header = header;

  for (const auto &detection : detections) {
    if (detection.empty()) {
      continue; // 跳过空的检测
    }

    vision_msgs::msg::Detection2D det;
    det.header = header;

    // 提取类别和置信度 (检测器存储在contour的第一个点)
    int class_id = static_cast<int>(detection[0].x);
    float confidence = detection[0].y;

    // 设置结果ID和分数
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = std::to_string(class_id);
    hypothesis.hypothesis.score = confidence;
    det.results.push_back(hypothesis);

    // TODO: 如果需要可以添加边界框信息

    detection_array.detections.push_back(det);
  }

  return detection_array;
}

} // namespace fruit_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fruit_detector::FruitDetector)