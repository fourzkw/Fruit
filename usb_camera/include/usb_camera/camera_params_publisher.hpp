#ifndef USB_CAMERA_CAMERA_PARAMS_PUBLISHER_HPP_
#define USB_CAMERA_CAMERA_PARAMS_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <string>

namespace usb_camera
{

class CameraParamsPublisher : public rclcpp::Node
{
public:
  explicit CameraParamsPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CameraParamsPublisher() = default;

private:
  void timer_callback();

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA_CAMERA_PARAMS_PUBLISHER_HPP_ 