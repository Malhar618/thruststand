cmake_minimum_required(VERSION 3.16) # Increased minimum version
cmake_policy(SET CMP0148 NEW)       # <<< ADDED: Suppress Python warning
project(drone_dynamixel_bridge)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Needed for tf2 conversions
find_package(DynamixelSDK REQUIRED)      # <<< MODIFIED: Use find_package, assuming system install or built underlay
find_package(Eigen3 REQUIRED)

# Executable: drone_dynamixel_bridge
add_executable(drone_dynamixel_bridge
  src/drone_dynamixel_bridge.cpp
)
ament_target_dependencies(drone_dynamixel_bridge
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  DynamixelSDK # <<< MODIFIED: Use package name found by find_package
  Eigen3
)
# Link Eigen3 explicitly if needed (often handled by ament_target_dependencies, but good practice)
target_link_libraries(drone_dynamixel_bridge Eigen3::Eigen)

# Executable: motor_stepup
add_executable(motor_stepup
  src/motor_stepup.cpp
)
ament_target_dependencies(motor_stepup
  rclcpp
  px4_msgs
)

# Executable: motor_stepdown
add_executable(motor_stepdown
  src/motor_stepdown.cpp
)
ament_target_dependencies(motor_stepdown
  rclcpp
  px4_msgs
)

# Executable: angular_state_logger
add_executable(angular_state_logger
  src/angular_state_logger.cpp
)
ament_target_dependencies(angular_state_logger
  rclcpp
  px4_msgs
)

# New Executable: performance_index (from performance_index.cpp)
add_executable(performance_index
  src/performance_index.cpp
)
ament_target_dependencies(performance_index
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)
target_link_libraries(performance_index Eigen3::Eigen) # <<< ADDED: Explicit Eigen link

# Install targets
install(TARGETS
  drone_dynamixel_bridge
  motor_stepup
  motor_stepdown
  angular_state_logger
  performance_index
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
