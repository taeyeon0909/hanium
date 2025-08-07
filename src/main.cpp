#include <rclcpp/rclcpp.hpp>
#include "coco/motion_command_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coco::MotionCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
