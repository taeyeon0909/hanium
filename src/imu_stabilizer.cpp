// src/imu_stabilizer.cpp
#include "coco/imu_stabilizer.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using std::placeholders::_1;

namespace coco {

ImuStabilizer::ImuStabilizer() : Node("imu_stabilizer")
{
  // 안전 임계/감속 파라미터
  pitch_slow_deg_ = declare_parameter("pitch_slow_deg", 10.0);
  pitch_stop_deg_ = declare_parameter("pitch_stop_deg", 50.0);
  roll_slow_deg_  = declare_parameter("roll_slow_deg",  8.0);
  roll_stop_deg_  = declare_parameter("roll_stop_deg", 20.0);
  slow_scale_     = declare_parameter("slow_scale", 0.3);

  // 경사 보정 파라미터
  keep_ground_speed_ = declare_parameter("keep_ground_speed", true);
  max_scale_         = declare_parameter("max_scale", 1.4);       // 최대 1.4배까지만 가속
  max_lin_speed_     = declare_parameter("max_lin_speed", 0.35);   // 최종 속도 상한
  roll_steer_gain_   = declare_parameter("roll_steer_gain", 0.0);  // 기본 0(꺼짐)
  compensate_in_slow_= declare_parameter("compensate_in_slow", true);

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_motion", 10, std::bind(&ImuStabilizer::on_cmd, this, _1));
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&ImuStabilizer::on_imu, this, _1));
  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_motion_stable", 10);

  RCLCPP_INFO(get_logger(), "ImuStabilizer started (slope compensation enabled)");
}

void ImuStabilizer::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);

  std::lock_guard<std::mutex> lk(imu_mtx_);
  roll_  = r;  // rad
  pitch_ = p;  // rad
  imu_ready_.store(true, std::memory_order_relaxed);
}

void ImuStabilizer::on_cmd(const geometry_msgs::msg::Twist::SharedPtr in)
{
  geometry_msgs::msg::Twist out = *in;

  if (imu_ready_.load(std::memory_order_relaxed)) {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    const double roll_deg  = std::abs(roll_  * 180.0 / M_PI);
    const double pitch_deg = std::abs(pitch_ * 180.0 / M_PI);

    // 1) 안전 임계 (정지/감속)
    const bool stop = (pitch_deg >= pitch_stop_deg_) || (roll_deg >= roll_stop_deg_);
    const bool slow = (pitch_deg >= pitch_slow_deg_) || (roll_deg >= roll_slow_deg_);

    if (stop) {
      out.linear.x = 0.0;
    } else {
      // 2) 경사 보정: 수평면 속도 유지 (v_out = v_in / cos(pitch))
      if (keep_ground_speed_ && std::abs(out.linear.x) > 1e-4) {
        // cos가 너무 작아지는 경우 과증폭 방지: scale ≤ max_scale_
        //   scale = 1 / max( cos(pitch), 1/max_scale_ )
        const double cosp   = std::cos(pitch_);
        const double scale  = 1.0 / std::max(cosp, 1.0 / max_scale_);
        out.linear.x *= scale;
      }

      // 3) 감속 영역이면 (선택) 추가 감속
      if (slow && compensate_in_slow_) {
        out.linear.x *= slow_scale_;
      } else if (slow && !compensate_in_slow_) {
        // 보정 대신 감속만 원하면 이렇게:
        out.linear.x = in->linear.x * slow_scale_;
      }

      // 4) 최종 상한 클램프
      out.linear.x = std::clamp(out.linear.x, -max_lin_speed_, max_lin_speed_);

      // 5) (옵션) 롤 보정 조향
      if (roll_steer_gain_ != 0.0 && std::abs(out.linear.x) > 1e-4) {
        out.angular.z += roll_steer_gain_ * roll_ * out.linear.x;
      }
    }

    // 디버그 프린트
    std::cout << "[ImuStabilizer] roll=" << roll_deg
              << "°, pitch=" << pitch_deg
              << " | IN lin=" << in->linear.x << ", ang=" << in->angular.z
              << " -> OUT lin=" << out.linear.x << ", ang=" << out.angular.z
              << std::endl;
  } else {
    std::cout << "[ImuStabilizer] IMU not ready | IN lin=" << in->linear.x
              << ", ang=" << in->angular.z
              << " -> OUT lin=" << out.linear.x
              << ", ang=" << out.angular.z << std::endl;
  }

  pub_cmd_->publish(out);
}

} // namespace coco
