#include "container_perception/leg_detector.hpp"
#include <rclcpp/rclcpp.hpp>

/*int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LegDetector>();

  // Use a MultiThreadedExecutor so callbacks can run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}