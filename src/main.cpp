#include <rclcpp/rclcpp.hpp>
#include "coco/motion_command_node.h"
#include "coco/imu_stabilizer.h"   // ★ 추가

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto motion = std::make_shared<coco::MotionCommandNode>();
  auto imu    = std::make_shared<coco::ImuStabilizer>();

  // 두 노드를 한 프로세스에서 돌리기
  rclcpp::executors::MultiThreadedExecutor exec; // 안전하게 멀티스레드 실행자 사용
  exec.add_node(motion);
  exec.add_node(imu);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

