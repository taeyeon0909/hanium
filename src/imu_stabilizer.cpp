#include "coco/imu_stabilizer.h"
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

namespace coco {

ImuStabilizer::ImuStabilizer() : Node("imu_stabilizer")
{
  // topics
  in_cmd_topic_  = declare_parameter<std::string>("in_cmd_topic",  "/cmd_motion");
  out_cmd_topic_ = declare_parameter<std::string>("out_cmd_topic", "/cmd_vel");
  imu_topic_     = declare_parameter<std::string>("imu_topic",     "/imu/data");
  imu_qos_       = declare_parameter<std::string>("imu_qos",       "best_effort"); // or "reliable"

  // thresholds (optional)
  pitch_slow_deg_ = declare_parameter("pitch_slow_deg", 10.0);
  pitch_stop_deg_ = declare_parameter("pitch_stop_deg", 50.0);
  roll_slow_deg_  = declare_parameter("roll_slow_deg",   8.0);
  roll_stop_deg_  = declare_parameter("roll_stop_deg",  20.0);
  slow_scale_     = declare_parameter("slow_scale",     0.3);

  // slope compensation
  keep_ground_speed_ = declare_parameter("keep_ground_speed", true);
  max_scale_         = declare_parameter("max_scale",        1.2);
  max_lin_speed_     = declare_parameter("max_lin_speed",    0.8);
  roll_steer_gain_   = declare_parameter("roll_steer_gain",  0.0);
  compensate_in_slow_= declare_parameter("compensate_in_slow", true);

  // QoS
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  if (imu_qos_ == "reliable") sensor_qos.reliable(); else sensor_qos.best_effort();
  sensor_qos.durability_volatile(); // 필요하면 유지

  // subs/pubs
  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      in_cmd_topic_, rclcpp::QoS(10), std::bind(&ImuStabilizer::on_cmd, this, _1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&ImuStabilizer::on_imu, this, _1));

  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(out_cmd_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
    "IMU Stabilizer: in=[%s] -> out=[%s], imu=[%s] (qos=%s) | keep= %s, max_scale=%.2f, vmax=%.2f, roll_gain=%.3f",
    in_cmd_topic_.c_str(), out_cmd_topic_.c_str(), imu_topic_.c_str(), imu_qos_.c_str(),
    keep_ground_speed_ ? "true":"false", max_scale_, max_lin_speed_, roll_steer_gain_);
}

void ImuStabilizer::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);

  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    roll_  = r;
    pitch_ = p;
  }
  if (!imu_ready_.exchange(true)) {
    RCLCPP_INFO(get_logger(), "IMU ready: starting compensation");
  }
}

void ImuStabilizer::on_cmd(const geometry_msgs::msg::Twist::SharedPtr in)
{
  geometry_msgs::msg::Twist out = *in;

  if (!imu_ready_.load(std::memory_order_relaxed)) {
    pub_cmd_->publish(out); // pass-through until IMU arrives
    return;
  }

  double roll, pitch;
  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    roll = roll_; pitch = pitch_;
  }

  // 1) 안전 임계
  const double roll_deg  = std::abs(roll  * 180.0 / M_PI);
  const double pitch_deg = std::abs(pitch * 180.0 / M_PI);
  const bool stop = (pitch_deg >= pitch_stop_deg_) || (roll_deg >= roll_stop_deg_);
  const bool slow = (pitch_deg >= pitch_slow_deg_) || (roll_deg >= roll_slow_deg_);

  if (stop) {
    out.linear.x = 0.0;
  } else {
    // 2) 경사 보정: 수평면 속도 유지
    if (keep_ground_speed_ && std::abs(out.linear.x) > 1e-4) {
      const double cosp  = std::cos(pitch);
      const double scale = 1.0 / std::max(std::abs(cosp), 1.0 / max_scale_); // ≤ max_scale_
      double v = out.linear.x * scale;
      // 최종 상한
      v = std::clamp(v, -max_lin_speed_, max_lin_speed_);
      out.linear.x = v;
    }

    // 3) 감속영역 처리
    if (slow) {
      if (compensate_in_slow_) out.linear.x *= slow_scale_;
      else                      out.linear.x = in->linear.x * slow_scale_;
    }

    // 4) 롤 기반 조향 보정(옵션)
    if (roll_steer_gain_ != 0.0 && std::abs(out.linear.x) > 1e-4) {
      out.angular.z += (-roll) * std::abs(out.linear.x) * roll_steer_gain_;
    }
  }

  pub_cmd_->publish(out);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
    "[ImuStabilizer] r=%.1f° p=%.1f° | IN lin=%.3f ang=%.3f -> OUT lin=%.3f ang=%.3f",
    roll_deg, pitch_deg, in->linear.x, in->angular.z, out.linear.x, out.angular.z);
}

} // namespace coco
