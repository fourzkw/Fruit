#ifndef FRUIT_DETECTOR_HPP_
#define FRUIT_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>  // 添加用于发布目标偏移量
#include <std_msgs/msg/int8_multi_array.hpp> // 添加用于订阅水果类型数组
#include "fruit_detector/openvino_detect.hpp"
#include <chrono>  // 添加时间相关头文件

namespace fruit_detector {

class FruitDetector : public rclcpp::Node {
public:
  explicit FruitDetector(const rclcpp::NodeOptions & options);
  virtual ~FruitDetector() = default;

private:
  // 图像订阅回调
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // 抓取信息回调
  void grabMsgCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
  
  // 将检测结果转换为ROS消息
  vision_msgs::msg::Detection2DArray createDetectionMessage(
    const std::vector<std::vector<cv::Point2f>>& detections,
    const std_msgs::msg::Header& header
  );
  
  // 参数
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string model_path_;
  float conf_threshold_;
  float iou_threshold_;
  
  // OpenVINO推理器
  std::unique_ptr<yolo::Inference> inference_;
  
  // ROS订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr grab_msg_subscription_; // 抓取消息订阅
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;  // 推理后图像发布器
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_offset_publisher_;  // 目标偏移量发布器
  
  // 相机信息
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  
  // 水果类型数组
  std::vector<int8_t> target_fruit_types_;
  
  // FPS计算相关
  std::chrono::time_point<std::chrono::steady_clock> last_frame_time_;
  float current_fps_;
  bool first_frame_;
};

} // namespace fruit_detector

#endif // FRUIT_DETECTOR_HPP_ 