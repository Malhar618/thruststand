# ACSL Thrust Stand
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

This repository implements the ROS2 node and firmware-level integration used to operate and control a high-fidelity thrust stand for evaluating adaptive control algorithms on small UAVs. The system supports moment cancellation via gimbal actuation (roll, pitch, yaw) using Dynamixel MX-106T motors and receives angular state feedback from a Pixhawk flight controller using the PX4-ROS2 bridge. The system is tightly coupled with MATLAB to perform genetic algorithm optimization of UAV control parameters.

This thrust stand project is part of a broader research initiative titled:

**"Genetic Algorithms and High-Fidelity Simulations for Tuning Model Reference Adaptive Control Systems"**

The primary objective is to demonstrate how closed-loop feedback data from high-precision flight simulations (via the thrust stand) can be used to automatically tune UAV control systems, such as PID controllers and adaptive inner-loop dynamics, using genetic algorithms.


## Repository Setup

To clone this repo with all the needed submodules you can run the command:

```bash
git clone --recurse-submodules https://github.com/andrealaffly/acsl-thrust-stand.git
```

## Build Instructions

### Building the Codebase


Once cloned, to build the codebase, run:

```bash
# Navigate to the repository
cd <path-to-cloned-repository>/acsl-thrust-stand/

# Make the build script executable
sudo chmod +x build-code.sh

# Run the build script
./build-code.sh
```

>[!NOTE]
> When prompted for password, use: `odroid`



### Cleaning the Build

To clean the built code:

```bash
# Navigate to the repository
cd <path-to-cloned-repository>/acsl-thrust-stand/

# Make the clean-code script executable
sudo chmod +x clean-code.sh

# Run the build script
./clean-code.sh
```

>[!CAUTION]
> This operation will permanently remove:
> - `install/`, `build/`, and `log/` directories
> - `tcp_server` executable from `src/comms/`
>
> Rebuild required after cleaning via `build-code.sh`.

---

## How to Use
Once the hardware setup is complete, follow these steps to launch the flight stack and begin testing.

### 1. Clone the Git Repository

Clone the repository if not already done.

### 2. Build drone_dynamixel_bridge  (if code was updated)

Navigate to the workspace and rebuild:

```bash
cd ~/ros2_thrust_ws
colcon build --packages-select drone_dynamixel_bridge
source install/setup.bash
source /opt/ros/galactic/setup.bash
```

### 3. Check Dynamixel Configuration

Using Dynamixel Wizard 2.0, ensure all motors are:
- Set to `current control mode`.
- Torque-enabled on all three axes (roll, pitch, yaw).
- Once verified, connect the motors to the Odroid via the U2D2 power board and USB hub.

### 4. Start the micro-ROS Agent

In a new terminal:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 5. Run the Dynamixel Bridge Node

In another terminal:

```bash
cd ~/ros2_thrust_ws
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 run drone_dynamixel_bridge drone_dynamixel_bridge
```

> **Note:** This node computes and sends torque commands. Allow some startup time.

### 6. (Optional) Launch the Flight Stack Node

Run this only if you intend to execute the full control loop including UAV rotor commands:

```bash
ros2 run flightstack flightstack
```

> ⚠️ **Warning:** This may spin up the drone’s rotors. Ensure the system is secure.

### 7. Run the Performance Index Node

To evaluate control system performance over the mission:

```bash
ros2 run drone_dynamixel_bridge performance_index
```

---

### Important Operational Notes

- The drone must be powered on and connected to receive odometry; rotors will not spin unless the flight stack node is running.
- The remote controller kill switch works independently. Power it on *before* powering the drone.
- Always start the micro-ROS agent before launching any ROS2 nodes.
- After pulling code changes or edits, rebuild with `colcon build` and re-source the environment.
- Connect the Pixhawk FTDI USB before the Dynamixel USB to maintain `/dev/ttyUSB0` ordering.
- The performance index node will automatically shut down and save its results to `performance_index.txt` after the mission duration.
"""
