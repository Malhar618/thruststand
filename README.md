# ROS2 Thrust Stand Control System for UAV Moment Cancellation and Performance Evaluation

This repository hosts the ROS2-based software system developed for the control and performance evaluation of a thrust stand designed for small multi-rotor unmanned aerial vehicles (UAVs). The primary focus of this work is to automatically compute and apply precise moment cancellation torques to maintain UAV attitude stability during dynamic testing. It also includes functionality for calculating a performance index to quantitatively evaluate flight control system effectiveness.

## Overview and Objectives

This project integrates rigid-body dynamics, advanced filtering techniques, and precise actuator control to:

- **Subscribe and Process Vehicle State Data:**  
  The ROS2 node acquires real-time UAV orientation (Euler angles) and angular velocity data directly from the PX4 flight controller through `/fmu/out/vehicle_odometry`. A numerical differentiator with built-in filtering is implemented to estimate angular accelerations from these velocities accurately.

- **Compute Dynamic Torques for Moment Cancellation:**  
  Utilizing known inertial properties of the UAV and thrust stand components (derived from CAD models), the system calculates torques required to neutralize unwanted moments produced by the rotating rings of the test stand, ensuring precise attitude stabilization.

- **Command Actuators (Dynamixel Motors):**  
  The calculated torques are translated into current commands suitable for Dynamixel MX-106 actuators operating in current-control mode. These commands are managed and sent using the Dynamixel SDK, ensuring rapid and reliable actuator responses.

- **Performance Index Evaluation:**  
  A dedicated ROS2 node computes a performance index based on the integrated squared difference (L2 norm) between actual and desired attitudes over the duration of a flight. This quantitative metric is essential for objectively assessing control system performance and guiding further adaptive controller tuning.

## Broader Impact

Automated tuning and accurate moment cancellation methods demonstrated in this project significantly advance UAV stability, maneuverability, and overall reliability. Enhancing these aspects supports safer and more effective applications in fields such as surveillance, search and rescue operations, environmental monitoring, precision agriculture, and beyond. Additionally, the methodologies developed are broadly applicable to other autonomous vehicles, providing a scalable approach to improving adaptive control strategies.

## Repository Structure
thruststand/
├── drone_dynamixel_bridge/
│   ├── src/
│   │   ├── drone_dynamixel_bridge.cpp  # Main ROS2 node for torque control
│   │   ├── performance_index.cpp       # Node calculating performance index
│   ├── CMakeLists.txt
│   └── package.xml
├── flightstack/  
├── dynamixel_sdk/  
└── px4_msgs/ 

## Hardware and Software Requirements

### Hardware:
- **Pixhawk 6x Pro:** Flight controller running PX4 autopilot firmware.
- **Dynamixel MX-106T actuators:** Operating in Current Control Mode (Protocol 2.0).
- **Odroid SBC:** Linux-based onboard computer running ROS2.
- **Custom Thrust Stand:** Designed for UAV moment measurement and cancellation.

### Software:
- **ROS2 Galactic**
- **PX4 Autopilot** (with `px4_msgs`)
- **Dynamixel SDK** (Protocol 2.0 support)
- **Eigen3** (linear algebra library)
- **MATLAB** (optional, for genetic algorithm-based tuning integrations)
- **TF2** (for quaternion and rotation transformations)

## Setup and Execution Instructions

### Initial ROS2 Environment Setup:
```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_thrust_ws
source install/setup.bash

