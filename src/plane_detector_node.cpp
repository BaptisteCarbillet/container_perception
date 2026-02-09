#include "plane_detector/plane_detector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaneDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
