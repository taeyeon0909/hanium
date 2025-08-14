#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

class JointCommandPublisher : public rclcpp::Node {
public:
  JointCommandPublisher() : Node("joint_command_publisher") {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_angles", 10,
      std::bind(&JointCommandPublisher::on_joint_state, this, std::placeholders::_1));
    pub0_ = create_publisher<std_msgs::msg::Float64>("/joint0_torque_controller/commands", 10);
    pub1_ = create_publisher<std_msgs::msg::Float64>("/joint1_torque_controller/commands", 10);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std_msgs::msg::Float64 cmd0, cmd1;
    // TODO: convert desired angle msg->torque setpoints
    pub0_->publish(cmd0);
    pub1_->publish(cmd1);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub0_, pub1_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
