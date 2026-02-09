#include <rclcpp/rclcpp.hpp>
#include "container_perception/container_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ContainerTracker>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
