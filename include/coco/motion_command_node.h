#ifndef COCO_MOTION_COMMAND_NODE_H_
#define COCO_MOTION_COMMAND_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace coco {

class MotionCommandNode : public rclcpp::Node {
public:
  MotionCommandNode();

private:
  void on_twist(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  const double max_forward_lin_ = 0.2;   // m/s forward
  const double max_backward_lin_ = -0.2; // m/s backward
  const double turn_lin_ = 0.1;          // m/s linear during turn
  const double turn_ang_ = 0.1;          // rad/s angular during turn
};

}  // namespace coco

#endif  // COCO_MOTION_COMMAND_NODE_H_
