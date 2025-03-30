#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <iostream>

// Include Eigen (make sure libeigen3-dev is installed)
#include <Eigen/Dense>

// For timer literals:
using namespace std::chrono_literals;

// --- Dynamixel configuration constants ---
static constexpr uint8_t  DXL_ID              = 2;               // Motor ID for roll axis
static constexpr char      DEVICENAME[]        = "/dev/ttyUSB1";    // Adjust this port as needed
static constexpr int       BAUDRATE            = 4000000;           // Baudrate for the MX-106T
static constexpr uint16_t  ADDR_TORQUE_ENABLE  = 64;              // Register for torque enable
static constexpr uint16_t  ADDR_GOAL_CURRENT    = 102;              // Register for goal current
static constexpr double      CURRENT_STEP        = 0.0045;         // 4.5 mA per step

// --- Theoretical Torque Equations: EqDynamics class ---
// (You might eventually move this class to its own header file.)
class EqDynamics {
public:
  EqDynamics() {
    // Initialize IJ matrix to zero.
    IJ_ = Eigen::Matrix3d::Zero();
  }

  // Returns the skew-symmetric matrix corresponding to the cross product with vector v.
  Eigen::Matrix3d crossProductArray(const Eigen::Vector3d& v) {
    Eigen::Matrix3d v_cross;
    v_cross <<   0,   -v.z(),  v.y(),
               v.z(),   0,    -v.x(),
              -v.y(),  v.x(),   0;
    return v_cross;
  }

  // Compute the rotated inertia matrix IJ = C_JLi * IJ_tilde * C_JLi^T.
  Eigen::Matrix3d computeInertia(const Eigen::Matrix3d& IJ_tilde, const Eigen::Matrix3d& C_JLi) {
    return C_JLi * IJ_tilde * C_JLi.transpose();
  }

  // Compute IJ * domega_JtoI^J.
  Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d& IJ, const Eigen::Vector3d& omega_dot) {
    return IJ * omega_dot;
  }

  // Compute omega1_cross * IJ * omega2.
  Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d& IJ, const Eigen::Vector3d& omega1, const Eigen::Vector3d& omega2) {
    Eigen::Matrix3d omega1_cross = crossProductArray(omega1);
    return omega1_cross * IJ * omega2;
  }

  // Main function: Compute the theoretical torque.
  // In this simplified version, we use only two terms:
  //   torque = IJ * (omega_JI_dot) + (omega_JI_cross) * IJ * (omega_JI)
  // The other terms (involving omega_LiJ) are omitted (assumed zero).
  Eigen::Vector3d computeTorque(const Eigen::Matrix3d& IJ_tilde, const Eigen::Matrix3d& C_JLi, 
                                const Eigen::Vector3d& omega_JI, 
                                const Eigen::Vector3d& omega_JI_dot) {
    Eigen::Matrix3d IJ = computeInertia(IJ_tilde, C_JLi);
    Eigen::Vector3d torque = computeIDomegaDt(IJ, omega_JI_dot) + computeOmegaCrossIomega(IJ, omega_JI, omega_JI);
    return torque;
  }

private:
  Eigen::Matrix3d IJ_;
};

// --- Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node
{
public:
  DroneDynamixelBridgeNode()
  : Node("drone_dynamixel_bridge_node"), prev_angular_velocity_(Eigen::Vector3d::Zero())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Drone Dynamixel Bridge Node...");

    // Subscribe to vehicle odometry topic.
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1)
    );

    // (Optional) Subscribe to attitude setpoint if available.
    setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", qos,
      std::bind(&DroneDynamixelBridgeNode::setpoint_callback, this, std::placeholders::_1)
    );

    // Set timer to run every 5ms (200 Hz).
    timer_ = this->create_wall_timer(5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this));

    // Initialize Dynamixel SDK.
    portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICENAME);
      return;
    }
    if (!portHandler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE);
      return;
    }
    // Enable torque control (assumed pre-configured for Torque Control Mode)
    write1Byte(DXL_ID, ADDR_TORQUE_ENABLE, 0);

    // Initialize our theoretical dynamics object.
    dynamics_ = std::make_shared<EqDynamics>();

    // Set constant matrices (IJ_tilde and C_JLi) for the thruststand.
    // These should be set to values obtained from your CAD or design. For now, we use example values.
    IJ_tilde_ = Eigen::Matrix3d::Zero();
    IJ_tilde_ << 1.0, 0.1, 0.2,
                 0.1, 1.0, 0.3,
                 0.2, 0.3, 1.0;
    // Example rotation matrix (C_JLi), e.g. a rotation about Z by 30°.
    double angle = 30.0 * M_PI / 180.0;
    C_JLi_ = Eigen::Matrix3d::Identity();
    C_JLi_(0,0) = std::cos(angle);  C_JLi_(0,1) = -std::sin(angle);
    C_JLi_(1,0) = std::sin(angle);  C_JLi_(1,1) = std::cos(angle);

    RCLCPP_INFO(this->get_logger(), "DroneDynamixelBridgeNode initialized.");
  }

  ~DroneDynamixelBridgeNode()
  {
    // Disable torque and close port on shutdown.
    write1Byte(DXL_ID, ADDR_TORQUE_ENABLE, 0);
    portHandler_->closePort();
  }

private:
  // --- ROS2 Subscribers and Timer ---
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  // Latest received messages.
  px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
  px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr latest_setpoint_msg_;

  // Dynamixel SDK pointers.
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  // Theoretical dynamics instance.
  std::shared_ptr<EqDynamics> dynamics_;

  // For numerical differentiation (store previous angular velocity)
  Eigen::Vector3d prev_angular_velocity_;
  // For time differentiation.
  rclcpp::Time prev_time_;

  // Constant matrices for the dynamics computation.
  Eigen::Matrix3d IJ_tilde_;
  Eigen::Matrix3d C_JLi_;

  // --- Callback: Update odometry message ---
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odometry_msg_ = msg;
    // Save current time for differentiation if not already set.
    if (!prev_time_.nanoseconds()) {
      prev_time_ = this->now();
      // Initialize previous angular velocity from odometry.
      prev_angular_velocity_ = Eigen::Vector3d(msg->angular_velocity[0],
                                               msg->angular_velocity[1],
                                               msg->angular_velocity[2]);
    }
  }

  // --- Callback: Update setpoint message (if available) ---
  void setpoint_callback(const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_setpoint_msg_ = msg;
  }

  // --- Timer Callback: Runs every 5ms ---
  void timer_callback()
  {
    // Copy latest messages in a thread-safe manner.
    px4_msgs::msg::VehicleOdometry::SharedPtr odom;
    px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr setpoint;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_odometry_msg_) {
        return;
      }
      odom = latest_odometry_msg_;
      setpoint = latest_setpoint_msg_;  // May be null if not available.
    }

    // Get current time.
    rclcpp::Time current_time = this->now();
    double dt = (current_time - prev_time_).seconds();
    if (dt <= 0) dt = 0.005;  // fallback to 5ms if dt not valid.
    prev_time_ = current_time;

    // Extract the actual attitude quaternion from odometry.
    double roll_actual, pitch_actual, yaw_actual;
    {
      tf2::Quaternion q_actual(odom->q[1], odom->q[2], odom->q[3], odom->q[0]);
      tf2::Matrix3x3 m_actual(q_actual);
      m_actual.getRPY(roll_actual, pitch_actual, yaw_actual);
    }

    // For dynamics computation, we need:
    // - omega_JI: angular velocity from odometry (as Eigen::Vector3d)
    // - omega_JI_dot: numerical derivative of angular velocity.
    Eigen::Vector3d omega_JI(odom->angular_velocity[0],
                               odom->angular_velocity[1],
                               odom->angular_velocity[2]);
    // Compute numerical derivative.
    Eigen::Vector3d omega_JI_dot = (omega_JI - prev_angular_velocity_) / dt;
    prev_angular_velocity_ = omega_JI;

    // For simplicity, assume omega_LiJ and its derivative are zero.
    Eigen::Vector3d omega_LiJ = Eigen::Vector3d::Zero();
    // Compute the theoretical torque.
    Eigen::Vector3d torque = dynamics_->computeTorque(IJ_tilde_, C_JLi_, omega_JI, omega_JI_dot);
    // Here we assume we are only interested in the roll (X) component.
    double roll_torque = torque.x();

    // Convert the computed torque (in N·m) to a Dynamixel Goal Torque register value.
    int goal_torque_reg = torque_to_mx106_goal_torque(roll_torque);

    // Write the computed goal torque to the Dynamixel motor.
    write2Byte(DXL_ID, ADDR_GOAL_CURRENT, static_cast<uint16_t>(goal_torque_reg));

    // Log output (throttled to 2 seconds)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5,
      "Roll omega: %.4f pitch omega: %.4f  yaw omega: %.4f", omega_JI[0], omega_JI[1], omega_JI[2]);
  }

  // --- Helper: Converts torque (N·m) to Dynamixel register value using piecewise conversion ---
  int torque_to_mx106_goal_torque(double torque_nm)
  {
    double currentA = 0.0;
    if (torque_nm >= 0.0) {
      currentA = 0.6370 * torque_nm + 0.0395;
    } else {
      currentA = 0.64390 * torque_nm - 0.1097;
    }
    double steps = currentA / CURRENT_STEP;
    int reg_value = 0;
    if (steps >= 0.0) {
      if (steps > 1023.0) steps = 1023.0;
      reg_value = static_cast<int>(std::round(steps));
    } else {
      double posSteps = -steps;
      if (posSteps > 1023.0) posSteps = 1023.0;
      reg_value = 1024 + static_cast<int>(std::round(posSteps));
    }
    return reg_value;
  }

  // --- Helper: Write 1 byte using Dynamixel SDK ---
  void write1Byte(uint8_t dxl_id, uint16_t addr, uint8_t value)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dxl_id, addr, value, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Comm fail write1Byte: %s",
                   packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error) {
      RCLCPP_ERROR(this->get_logger(), "DXL error write1Byte: %s",
                   packetHandler_->getRxPacketError(dxl_error));
    }
  }

  // --- Helper: Write 2 bytes using Dynamixel SDK ---
  void write2Byte(uint8_t dxl_id, uint16_t addr, uint16_t value)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, dxl_id, addr, value, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Comm fail write2Byte: %s",
                   packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error) {
      RCLCPP_ERROR(this->get_logger(), "DXL error write2Byte: %s",
                   packetHandler_->getRxPacketError(dxl_error));
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneDynamixelBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

