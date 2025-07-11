#include "fruit_detector/fruit_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<fruit_detector::FruitDetector>(options);
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
} 