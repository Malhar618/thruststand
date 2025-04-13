cmake_minimum_required(VERSION 3.16)
project(drone_dynamixel_bridge)

# If you're targeting Python 3 only, and on new CMake versions, set policy CMP0148 to NEW
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)
# Modern Python 3 module (replaces old FindPythonInterp/FindPythonLibs)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# drone_dynamixel_bridge executable
add_executable(drone_dynamixel_bridge src/drone_dynamixel_bridge.cpp)
ament_target_dependencies(drone_dynamixel_bridge
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  dynamixel_sdk
  Eigen3
)
target_link_libraries(drone_dynamixel_bridge Eigen3::Eigen)

# motor_stepup executable
add_executable(motor_stepup src/motor_stepup.cpp)
ament_target_dependencies(motor_stepup
  rclcpp
  px4_msgs
)

# motor_stepdown executable
add_executable(motor_stepdown src/motor_stepdown.cpp)
ament_target_dependencies(motor_stepdown
  rclcpp
  px4_msgs
)

# angular_state_logger executable
add_executable(angular_state_logger src/angular_state_logger.cpp)
ament_target_dependencies(angular_state_logger
  rclcpp
  px4_msgs
)

# performance_index executable
add_executable(performance_index src/performance_index.cpp)
ament_target_dependencies(performance_index
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)
target_link_libraries(performance_index Eigen3::Eigen)

# Install the targets
install(TARGETS
  drone_dynamixel_bridge
  motor_stepup
  motor_stepdown
  angular_state_logger
  performance_index
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
