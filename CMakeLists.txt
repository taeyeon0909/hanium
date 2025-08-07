cmake_minimum_required(VERSION 3.5)
project(coco)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(motion_command_node
  src/motion_command_node.cpp
  src/main.cpp
)
ament_target_dependencies(motion_command_node
  rclcpp
  geometry_msgs
)
install(TARGETS motion_command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
