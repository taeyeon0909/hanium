#pragma once
#include <mutex>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coco {

// 최신 IMU 값을 어디서든 쉽게 가져다 쓰기 위한 작은 버퍼
class ImuStore {
public:
  static ImuStore& instance() {
    static ImuStore s; return s;
  }
  void set(const sensor_msgs::msg::Imu& m) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_ = m;
  }
  std::optional<sensor_msgs::msg::Imu> get() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_;
  }
private:
  mutable std::mutex mtx_;
  std::optional<sensor_msgs::msg::Imu> last_;
};

class ImuSubscriber : public rclcpp::Node {
public:
  explicit ImuSubscriber(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("imu_subscriber", opts)
  {
    declare_parameter<std::string>("imu_topic", "/imu");
    const auto topic = get_parameter("imu_topic").as_string();

    // 퍼블리셔(/imu)가 RELIABLE 이므로 구독도 RELIABLE로 맞춤
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      topic, qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg){
        coco::ImuStore::instance().set(*msg);
        RCLCPP_DEBUG(this->get_logger(), "IMU t=%.3f",
                     msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
      });
    RCLCPP_INFO(this->get_logger(), "Subscribing IMU from [%s] (RELIABLE)", topic.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

} // namespace coco
