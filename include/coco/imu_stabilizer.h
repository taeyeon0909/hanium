#ifndef COCO_IMU_STABILIZER_H_
#define COCO_IMU_STABILIZER_H_

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

  // 임계/감속 파라미터
  double pitch_slow_deg_;
  double pitch_stop_deg_;
  double roll_slow_deg_;
  double roll_stop_deg_;
  double slow_scale_;

  // ===== 경사 보정 파라미터 =====
  bool   keep_ground_speed_;   // true면 수평면 속도 유지 보정 켬
  double max_scale_;           // 보정 최대 배율 (ex. 1.4 → 최대 1.4배)
  double max_lin_speed_;       // 최종 선속도 상한 (안전)
  double roll_steer_gain_;     // 롤 보정 조향 게인 (rad 조향 / (rad * m/s)). 0이면 비활성
  bool   compensate_in_slow_;  // 감속 영역에서도 보정 적용할지 (기본 true)
};

}  // namespace coco

#endif  // COCO_IMU_STABILIZER_H_
