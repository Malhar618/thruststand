# ROS2-Based Adaptive Control Thrust Stand

This repository implements the ROS2 node and firmware-level integration used to operate and control a high-fidelity thrust stand for evaluating adaptive control algorithms on small UAVs. The system supports moment cancellation via gimbal actuation (roll, pitch, yaw) using Dynamixel MX-106T motors and receives angular state feedback from a Pixhawk flight controller using the PX4 ROS2 bridge. The system is tightly coupled with MATLAB to perform genetic algorithm optimization of UAV control parameters.

This thrust stand project is part of a broader research initiative titled:

**"Genetic Algorithms and High-Fidelity Simulations for Tuning Model Reference Adaptive Control Systems"**

The primary objective is to demonstrate how closed-loop feedback data from high-precision flight simulations (via the thrust stand) can be used to automatically tune UAV control systems, such as PID controllers and adaptive inner-loop dynamics, using genetic algorithms.

---

## Hardware Used

- **Pixhawk 6X Pro**: Sends odometry and angular velocity feedback.
- **Dynamixel MX-106T (x3)**: Roll, pitch, and yaw actuation using protocol 2.0 over USB (torque/current mode).
- **Odroid (running ROS2 Galactic)**: Receives sensor data, computes torque commands, actuates gimbals.
- **3-DOF Gimbal-Mounted UAV Stand**: Allows for realistic moment propagation between UAV and rings.
- **MATLAB + Genetic Algorithms Toolbox**: Performs tuning of control systems based on flight stand behavior.

---

## ROS2 Node Structure

Implemented in the `drone_dynamixel_bridge` ROS2 package (C++).
- Subscribes to `/fmu/out/vehicle_odometry` to get position, orientation, and angular velocity.
- Computes filtered angular acceleration using low-pass differentiator filters.
- Computes net required cancellation torque using inertia matrices and rigid body dynamics.
- Inverts the Jacobian to convert torque into gimbal torques (for roll, pitch, yaw).
- Sends current commands to Dynamixel motors via GroupSyncWrite.

The control architecture uses a moment-of-inertia-based torque cancellation model.

---

## Building the Workspace:

```bash
cd ~/ros2_thrust_ws
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
```

Make sure you have the following packages built in your workspace:
- `px4_msgs`
- `flightstack`
- `drone_dynamixel_bridge`
- `dynamixel_sdk`

> Note: You may need to clone or re-add submodules (like `px4_msgs`, `flightstack`, or `dynamixel_sdk`) if not already present.

---

## Running the System

1. Start the **micro-ROS agent** (in a new terminal):
```bash
cd ~/ros2_thrust_ws
./start_uxrce.sh
```

2. Start the **Dynamixel Bridge Node**:
```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 run drone_dynamixel_bridge drone_dynamixel_bridge
```

3. Start the **Flightstack Node** (in another terminal):
```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 run flightstack flightstack
```

You should begin seeing filtered angular states, torques, and current commands in real time.

---

## Notes on Customization

- **Torque Conversion**: Calibrated based on torque-to-current fit for MX-106T.
- **Filter Gains**: Angular acceleration filters can be tuned in code (`wn`, `zeta`).
- **Inertia Matrices**: These are manually specified in code based on CAD-derived values.

---

## Broader Impact

This system allows researchers to evaluate flight controller designs *before deployment on real drones*, reducing risk and enabling faster iteration. With the real-time feedback and MATLAB interface, the system provides a robust closed-loop platform for optimization and benchmarking of adaptive or PID-based control architectures.

Research from this project may inform the design of fault-tolerant and adaptive control for Navy UAV platforms and is intended for demonstration at NAVAIR in May 2025.

---

## Results

Measured data, filtered states, and actuator logs will be published to this repository after testing. Refer to the `/logs` directory for CSV logs and the `/docs` directory for visuals, equations, and results.

