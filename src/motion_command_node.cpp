#include "coco/motion_command_node.h"
#include <cmath>
#include <iostream>

namespace coco {

MotionCommandNode::MotionCommandNode()
: Node("motion_command_node")
{
  // 안전 기본값: /cmd_vel_raw -> /cmd_motion (루프 방지)
  in_cmd_topic_  = this->declare_parameter<std::string>("in_cmd_topic",  "/cmd_vel_raw");
  out_cmd_topic_ = this->declare_parameter<std::string>("out_cmd_topic", "/cmd_motion");
  mode_          = this->declare_parameter<std::string>("mode", "quantized");

  max_forward_lin_  = this->declare_parameter<double>("max_forward_lin",  0.6);
  max_backward_lin_ = this->declare_parameter<double>("max_backward_lin", -0.3);
  turn_ang_         = this->declare_parameter<double>("turn_ang",         1.0);
  turn_lin_         = this->declare_parameter<double>("turn_lin",         0.0);
  lin_deadband_     = this->declare_parameter<double>("lin_deadband",     1e-3);
  ang_deadband_     = this->declare_parameter<double>("ang_deadband",     1e-3);

  lin_step_         = this->declare_parameter<double>("lin_step",         0.5);
  ang_step_         = this->declare_parameter<double>("ang_step",         1.0);
  db_ratio_         = this->declare_parameter<double>("db_ratio",         0.2);

  if (max_backward_lin_ > 0.0) {
    RCLCPP_WARN(get_logger(), "max_backward_lin (%.3f) must be negative. Forcing negative.", max_backward_lin_);
    max_backward_lin_ = -std::fabs(max_backward_lin_);
  }

  sub_ = create_subscription<geometry_msgs::msg::Twist>(
      in_cmd_topic_, rclcpp::QoS(10),
      std::bind(&MotionCommandNode::on_twist, this, std::placeholders::_1));

  pub_ = create_publisher<geometry_msgs::msg::Twist>(out_cmd_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "[MotionCommandNode] in=%s -> out=%s | mode=%s",
              in_cmd_topic_.c_str(), out_cmd_topic_.c_str(), mode_.c_str());
  std::cout << "[MotionCommandNode] initialized: custom mapping active" << std::endl;
}

void MotionCommandNode::on_twist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double lin = msg->linear.x;
  const double ang = msg->angular.z;
  geometry_msgs::msg::Twist out{};

  if (mode_ == "passthrough") {
    out = *msg;
  }
  else if (mode_ == "quantized") {
    auto quant = [](double v, double step, double db_ratio){
      const double db = std::fabs(step) * db_ratio;
      if (std::fabs(v) < db) return 0;
      return v > 0.0 ? +1 : -1;
    };
    const int L = quant(lin, lin_step_, db_ratio_);
    const int A = quant(ang, ang_step_, db_ratio_);

    out.linear.x  = (L > 0) ? max_forward_lin_ : (L < 0 ? max_backward_lin_ : 0.0);
    out.angular.z = (A > 0) ? +turn_ang_      : (A < 0 ? -turn_ang_        : 0.0);
    if (A != 0 && L == 0) out.linear.x = turn_lin_;
  }
  else { // sign
    const bool lin_active = std::fabs(lin) > lin_deadband_;
    const bool ang_active = std::fabs(ang) > ang_deadband_;
    out.linear.x  = lin_active ? ((lin > 0.0) ? max_forward_lin_ : max_backward_lin_) : 0.0;
    out.angular.z = ang_active ? ((ang > 0.0) ? +turn_ang_      : -turn_ang_)        : 0.0;
    if (ang_active && !lin_active) out.linear.x = turn_lin_;
  }

  pub_->publish(out);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
    "[MotionCommandNode] mapped -> lin.x=%.3f ang.z=%.3f (mode=%s)",
    out.linear.x, out.angular.z, mode_.c_str());
}

} // namespace coco
