#include "coco/imu_stabilizer.h"
#include <algorithm>
#include <cmath>

namespace coco {

static inline void quatToRPY(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw){
  // ZYX 순서 기준
  // 참고: tf2::Quaternion -> Matrix3x3(q).getRPY(roll,pitch,yaw) 와 동일 로직
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tq);
  m.getRPY(roll, pitch, yaw);
}

ImuStabilizer::ImuStabilizer()
: rclcpp::Node("imu_stabilizer")
{
  // ----- 토픽/동작 파라미터 -----
  // 입력 cmd, 출력 cmd, imu 토픽
  std::string in_cmd_topic  = this->declare_parameter<std::string>("in_cmd_topic",  "/cmd_vel_in");
  std::string out_cmd_topic = this->declare_parameter<std::string>("out_cmd_topic", "/cmd_vel");
  std::string imu_topic     = this->declare_parameter<std::string>("imu_topic",     "/imu");

  // 보정 동작 관련(기본값은 무난하게 설정)
  keep_ground_speed_ = this->declare_parameter<bool>("keep_ground_speed", true); // 수평면 속도 유지
  max_scale_         = this->declare_parameter<double>("max_scale", 1.3);        // 최대 1.3배까지만 증속
  max_lin_speed_     = this->declare_parameter<double>("max_lin_speed", 0.8);    // 최종 선속도 상한 [m/s]
  roll_steer_gain_   = this->declare_parameter<double>("roll_steer_gain", 0.0);  // 0이면 조향 보정 X
  compensate_in_slow_= this->declare_parameter<bool>("compensate_in_slow", true);// (임계/감속 안쓰지만 호환 유지)

  // 임계/감속 파라미터는 요청대로 사용하지 않음(필드는 남겨둠)
  pitch_slow_deg_ = this->declare_parameter<double>("pitch_slow_deg",  999.0);
  pitch_stop_deg_ = this->declare_parameter<double>("pitch_stop_deg",  999.0);
  roll_slow_deg_  = this->declare_parameter<double>("roll_slow_deg",   999.0);
  roll_stop_deg_  = this->declare_parameter<double>("roll_stop_deg",   999.0);
  slow_scale_     = this->declare_parameter<double>("slow_scale",      1.0);

  // ----- QoS -----
  auto sensor_qos = rclcpp::SensorDataQoS().best_effort().keep_last(50);
  auto default_qos = rclcpp::SystemDefaultsQoS();

  // ----- 구독/퍼블리셔 -----
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      std::bind(&ImuStabilizer::on_imu, this, std::placeholders::_1));

  sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      in_cmd_topic, default_qos,
      std::bind(&ImuStabilizer::on_cmd, this, std::placeholders::_1));

  pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(out_cmd_topic, default_qos);

  RCLCPP_INFO(this->get_logger(),
              "IMU Stabilizer: in_cmd=[%s] -> out_cmd=[%s], imu=[%s], keep_ground_speed=%s, max_scale=%.2f, vmax=%.2f, roll_steer_gain=%.3f",
              in_cmd_topic.c_str(), out_cmd_topic.c_str(), imu_topic.c_str(),
              keep_ground_speed_ ? "true":"false", max_scale_, max_lin_speed_, roll_steer_gain_);
}

void ImuStabilizer::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double r, p, y;
  quatToRPY(msg->orientation, r, p, y);

  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    roll_  = r;
    pitch_ = p;
  }
  imu_ready_.store(true, std::memory_order_relaxed);
}

void ImuStabilizer::on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // IMU가 아직 없으면 원본 패스
  if (!imu_ready_.load(std::memory_order_relaxed)) {
    pub_cmd_->publish(*msg);
    return;
  }

  double roll, pitch;
  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    roll  = roll_;
    pitch = pitch_;
  }

  geometry_msgs::msg::Twist out = *msg;

  // ===== 1) 수평면 지상 속도 유지 보정 =====
  // 경사면에서 실제 지상 전진속도 ~= v_body * cos(pitch)
  // => 지상 기준 속도 유지하려면 v_out = v_in / cos(pitch)
  if (keep_ground_speed_) {
    const double cp = std::cos(pitch);
    // 너무 가파른 각도에서 무한증속 방지: |cp|가 너무 작으면 하한을 둔다.
    const double cp_safe = std::clamp(std::abs(cp), 0.3, 1.0); // 0.3~1.0
    double scale = 1.0 / cp_safe;

    // 과도한 증속 제한
    scale = std::min(scale, max_scale_);

    // 전/후진 모두 동일하게 크기 보정 (부호 유지)
    auto scale_with_sign_limit = [&](double vin){
      const double sign = (vin >= 0.0) ? 1.0 : -1.0;
      double v = std::abs(vin) * scale;
      // 최종 속도 상한
      v = std::min(v, max_lin_speed_);
      return sign * v;
    };

    out.linear.x = scale_with_sign_limit(out.linear.x);

    // (선택) 측면 속도도 쓰는 플랫폼이라면 Y축도 동일 원리로 보정
    out.linear.y = scale_with_sign_limit(out.linear.y);
  }

  // ===== 2) 롤 기반 조향 보정(옵션) =====
  // 경사면에서 좌/우로 쏠리는 현상을 줄이기 위해,
  // roll(+) → 오른쪽으로 기울었을 때 왼쪽으로 약간 조향.
  if (std::abs(roll_steer_gain_) > 1e-6) {
    const double v_mag = std::hypot(out.linear.x, out.linear.y);
    out.angular.z += (-roll) * v_mag * roll_steer_gain_;
  }

  pub_cmd_->publish(out);
}

} // namespace coco
