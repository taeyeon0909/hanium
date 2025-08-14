cmake_minimum_required(VERSION 3.8)
project(coco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# motion_command_node
add_executable(motion_command_node
  src/motion_command_node.cpp
  src/motion_command_main.cpp
)
target_include_directories(motion_command_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motion_command_node
  rclcpp geometry_msgs sensor_msgs)

# imu_stabilizer
add_executable(imu_stabilizer
  src/imu_stabilizer.cpp
  src/imu_stabilizer_main.cpp
)
target_include_directories(imu_stabilizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(imu_stabilizer
  rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  motion_command_node
  imu_stabilizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY include/
  DESTINATION include)

ament_package()
