#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

namespace coco {

class MotionCommandNode : public rclcpp::Node {
public:
  MotionCommandNode();

private:
  void on_twist(const geometry_msgs::msg::Twist::SharedPtr msg);

  // I/O
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_;
  std::string in_cmd_topic_;
  std::string out_cmd_topic_;

  // mapping: "quantized"(default) | "sign" | "passthrough"
  std::string mode_;

  // common mapping params
  double max_forward_lin_{0.6};   // m/s
  double max_backward_lin_{-0.3}; // m/s (음수)
  double turn_ang_{1.0};          // rad/s
  double turn_lin_{0.0};          // 회전만 할 때 살짝 전진

  // sign mode deadband
  double lin_deadband_{1e-3};
  double ang_deadband_{1e-3};

  // quantized mode
  double lin_step_{0.5};
  double ang_step_{1.0};
  double db_ratio_{0.2};
};

} // namespace coco
