cmake_minimum_required(VERSION 3.5)
project(coco)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) ament_cmake를 가장 먼저 찾는다 
find_package(ament_cmake REQUIRED)

# 2) 의존 패키지
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# 3) 실행파일 (한 프로세스에서 두 노드 실행)
add_executable(coco_bringup
  src/motion_command_node.cpp
  src/imu_stabilizer.cpp
  src/main.cpp
)

# 4) ament_target_dependencies
ament_target_dependencies(coco_bringup
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# 5) 설치
install(TARGETS coco_bringup
  DESTINATION lib/${PROJECT_NAME}
)

# 6) ament 패키지 마무리
ament_package()

