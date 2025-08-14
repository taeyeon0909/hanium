#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class IkSolverNode : public rclcpp::Node {
public:
  IkSolverNode() : Node("ik_solver_node") {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_motion", 10,
      std::bind(&IkSolverNode::on_motion, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_angles", 10);
    // TODO: load robot model, obstacle map
  }

private:
  void on_motion(const geometry_msgs::msg::Twist::SharedPtr msg) {
    sensor_msgs::msg::JointState out;
    // TODO: compute IK including obstacle avoidance
    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkSolverNode>());
  rclcpp::shutdown();
  return 0;
}
