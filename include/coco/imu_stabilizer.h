#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <atomic>
#include <mutex>

namespace coco {

class ImuStabilizer : public rclcpp::Node {
public:
  ImuStabilizer();

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_cmd_;

  std::atomic<bool> imu_ready_{false};
  std::mutex imu_mtx_;
  double roll_{0.0};   // rad
  double pitch_{0.0};  // rad

  // thresholds (optional safety)
  double pitch_slow_deg_;
  double pitch_stop_deg_;
  double roll_slow_deg_;
  double roll_stop_deg_;
  double slow_scale_;

  // slope compensation
  bool   keep_ground_speed_;
  double max_scale_;
  double max_lin_speed_;
  double roll_steer_gain_;
  bool   compensate_in_slow_;

  // QoS & topic params (read only at start)
  std::string in_cmd_topic_;
  std::string out_cmd_topic_;
  std::string imu_topic_;
  std::string imu_qos_; // "best_effort"|"reliable"
};

}  // namespace coco
