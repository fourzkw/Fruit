#include "usb_camera/camera_params_publisher.hpp"

#include <memory>
#include <string>

namespace usb_camera
{

CameraParamsPublisher::CameraParamsPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("camera_params_publisher", options)
{
  // Declare parameters
  this->declare_parameter("camera_name", "head_camera");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("frame_id", "camera_optical_frame");
  this->declare_parameter("publish_rate", 10.0);  // Hz

  // Get parameters
  auto camera_name = this->get_parameter("camera_name").as_string();
  auto camera_info_url = this->get_parameter("camera_info_url").as_string();
  auto frame_id = this->get_parameter("frame_id").as_string();
  auto publish_rate = this->get_parameter("publish_rate").as_double();
  
  RCLCPP_INFO(
    this->get_logger(), "Starting Camera Parameters Publisher for %s", camera_name.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Camera info URL: %s", camera_info_url.c_str());

  // Create publisher
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_name + "/camera_info", 10);
  
  // Set up the camera info manager
  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, camera_name, camera_info_url);
  
  // Load camera info
  if (camera_info_manager_->isCalibrated()) {
    camera_info_ = camera_info_manager_->getCameraInfo();
    camera_info_.header.frame_id = frame_id;
    RCLCPP_INFO(this->get_logger(), "Camera is calibrated");
  } else {
    RCLCPP_WARN(this->get_logger(), "Camera is not calibrated");
  }
  
  // Create timer for publishing camera info
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
    std::bind(&CameraParamsPublisher::timer_callback, this));
}

void CameraParamsPublisher::timer_callback()
{
  // Update timestamp
  camera_info_.header.stamp = this->now();
  
  // Publish camera info
  camera_info_pub_->publish(camera_info_);
}

}  // namespace usb_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera::CameraParamsPublisher)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<usb_camera::CameraParamsPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 