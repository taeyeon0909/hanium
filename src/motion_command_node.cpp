// src/motion_command_node.cpp

#include "coco/motion_command_node.h"
#include <iostream>    // for std::cout

namespace coco {

MotionCommandNode::MotionCommandNode()
: Node("motion_command_node")
{
  sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10, //10hz
    std::bind(&MotionCommandNode::on_twist, this, std::placeholders::_1));

  pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_motion",
    10); //10hz

  // 초기화 메시지
  std::cout << "[MotionCommandNode] initialized: custom mapping active" << std::endl;
}

void MotionCommandNode::on_twist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::Twist out;
  // 1) 매번 0.0으로 초기화
  out.linear.x  = 0.0;
  out.angular.z = 0.0;

  // 2) 입력 sign에 따라 무조건 전진 or 후진
  if ((msg->linear.x == 0.5 && msg->angular.z == 1.0) ) { //i
    out.angular.z =  turn_ang_;
  } 
  else if (msg->linear.x == 0.5 && msg->angular.z == 0.0) { //k
    out.linear.x = max_forward_lin_; 
  }
  else if (msg->linear.x == 0.0 && msg->angular.z ==  0.0) { //u
     out.linear.x = max_backward_lin_;
  }
  else if (msg->linear.x == 0.5 && msg->angular.z == -1.0) { //o
    out.angular.z = -turn_ang_;
  }

  // 3) 순수 회전(전진/후진 입력 없을 때)만 처리
  if (msg->linear.x == 0.0 && msg->angular.z != 0.0) {
    // 여기서도 _override_ 할당이 일어나므로 앞뒤 매핑에 영향 안 줌
    out.linear.x  = turn_lin_;  
    out.angular.z = (msg->angular.z > 0.0 ? turn_ang_ : -turn_ang_);
  }


  // 디버깅용 로그
  std::cout << "[MotionCommandNode] Mapped cmd -> "
            << "linear.x: " << out.linear.x
            << ", angular.z: " << out.angular.z
            << std::endl;

  pub_->publish(out);
}


}  // namespace coco
