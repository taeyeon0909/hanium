#include <rclcpp/rclcpp.hpp>
#include "coco/imu_stabilizer.h"
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coco::ImuStabilizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
