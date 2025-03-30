cmake_minimum_required(VERSION 3.8)
project(drone_dynamixel_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

# Existing executable target for drone_dynamixel_bridge.cpp
add_executable(drone_dynamixel_bridge
  src/drone_dynamixel_bridge.cpp
)
ament_target_dependencies(drone_dynamixel_bridge
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  dynamixel_sdk
  Eigen3
)

# Existing executable target for motor_stepup.cpp
add_executable(motor_stepup
  src/motor_stepup.cpp
)
ament_target_dependencies(motor_stepup
  rclcpp
  px4_msgs
)

# New executable target for motor_stepdown.cpp
add_executable(motor_stepdown
  src/motor_stepdown.cpp
)
ament_target_dependencies(motor_stepdown
  rclcpp
  px4_msgs
)

install(TARGETS
  drone_dynamixel_bridge
  motor_stepup
  motor_stepdown
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
