#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp> // For body rates
#include <dynamixel_sdk/dynamixel_sdk.h>  // SDK for Dynamixel
#include <chrono>
#include <mutex>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <thread> // For std::this_thread::sleep_for

using namespace std::chrono_literals;

// --- Dynamixel Configuration (Write-only) ---
const uint8_t DXL_ID_ROLL = 2;
const uint8_t DXL_ID_PITCH = 1;
const uint8_t DXL_ID_YAW = 3;
const std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
const char* DEVICENAME = "/dev/ttyUSB1"; // VERIFY that this port is connected to the Dynamixels
const int BAUDRATE = 4000000;
const float PROTOCOL_VERSION = 2.0;
const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_GOAL_CURRENT = 102;
const uint16_t LEN_GOAL_CURRENT = 2;
const double CURRENT_STEP = 0.00269; // 2.69 mA per step
const double DEFAULT_CURRENT_LIMIT_STEPS = 1193.0; // Approximately 3.21 A

// --- Filter Differentiator Class ---
// This class implements a simple 2nd order filter (with Euler integration) that can be used to compute a filtered derivative.
class FilterDiff {
public:
  FilterDiff(double wn, double zeta)
    : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), initialized_(false) {}
  void update(double input, double dt) {
    if (dt <= 1e-9)
      return;
    if (!initialized_) {
      x1_ = input;
      x2_ = 0.0;
      initialized_ = true;
    } else {
      double x1_dot = x2_;
      double x2_dot = -wn_ * wn_ * x1_ - 2.0 * zeta_ * wn_ * x2_ + wn_ * wn_ * input;
      x1_ += x1_dot * dt;
      x2_ += x2_dot * dt;
    }
  }
  double getFilteredDerivative() const { return x2_; }
  void reset(double initial_value = 0.0) { x1_ = initial_value; x2_ = 0.0; initialized_ = true; }
private:
  double wn_, zeta_, x1_, x2_;
  bool initialized_;
};

// --- Dynamics Equations Class ---
class EqDynamics {
public:
  EqDynamics() {}
  // Returns the cross-product (skew-symmetric) matrix for vector v.
  Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) const {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
  }
  // Computes I * (angular acceleration).
  Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &I, const Eigen::Vector3d &omega_dot) const {
    return I * omega_dot;
  }
  // Computes omega cross (I * omega)
  Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &I, const Eigen::Vector3d &omega1, const Eigen::Vector3d &omega2) const {
    return omega1.cross(I * omega2);
  }
  // Computes a simplified torque as the sum of two terms:
  //  I * (filtered angular acceleration)  +  (raw angular velocity cross (I * raw angular velocity)).
  // Then transforms it using the given transform matrix CJLi.
  Eigen::Vector3d compute_torque_simplified_mixed(const Eigen::Matrix3d &ILi_inLi,
                                                  const Eigen::Matrix3d &CJLi,
                                                  const Eigen::Vector3d &omega_LiI_inLi_raw,
                                                  const Eigen::Vector3d &omega_dot_LiI_inLi_filt) const {
    Eigen::Vector3d term1 = computeIDomegaDt(ILi_inLi, omega_dot_LiI_inLi_filt);
    Eigen::Vector3d term2 = computeOmegaCrossIomega(ILi_inLi, omega_LiI_inLi_raw, omega_LiI_inLi_raw);
    return CJLi * (term1 + term2);
  }
};

// --- Drone Dynamixel Bridge Node ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode()
    : Node("drone_dynamixel_bridge_node"),
      // Initialize inertia tensors (example values; adjust if needed and ensure correct reference frame)
      I_CUAV_((Eigen::Matrix3d() << 0.010679, -0.000001, -0.000072,
                                   -0.000001, 0.017318, 0.000007,
                                   -0.000072, 0.000007, 0.026610).finished()),
      I_CRB_((Eigen::Matrix3d() << 0.000847, -0.000004, -0.000010,
                                  -0.000004, 0.098249, 0.000000,
                                  -0.000010, 0.000000, 0.098096).finished()),
      I_CPR_((Eigen::Matrix3d() << 0.640302,  0.000043,  0.000078,
                                  0.000043,  0.570491, -0.000025,
                                  0.000078, -0.000025,  1.200050).finished()),
      I_CYR_((Eigen::Matrix3d() << 2.384163, -0.000049, -0.000104,
                                  -0.000049, 1.110062,  0.000781,
                                  -0.000104, 0.000781,  1.286313).finished())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing DroneDynamixelBridgeNode...");

    // Filter parameters for differentiating the body rates (p, q, r)
    double filter_omega_p = 70.0, filter_zeta_p = 0.7;
    double filter_omega_q = 70.0, filter_zeta_q = 0.7;
    double filter_omega_r = 70.0, filter_zeta_r = 0.7;

    // Create subscriptions with best-effort QoS
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1));
    angular_velocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", qos,
        std::bind(&DroneDynamixelBridgeNode::angular_velocity_callback, this, std::placeholders::_1));

    // Timer at 200 Hz (every 5 ms)
    timer_ = this->create_wall_timer(5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this));
    prev_time_ = this->now();

    // Initialize Dynamixel SDK
    if (!initialize_dynamixels()) {
      throw std::runtime_error("Dynamixel initialization failed");
    }
    dynamics_ = std::make_shared<EqDynamics>();

    // Create filter objects for body rate differentiation
    fd_p_deriv_ = std::make_unique<FilterDiff>(filter_omega_p, filter_zeta_p);
    fd_q_deriv_ = std::make_unique<FilterDiff>(filter_omega_q, filter_zeta_q);
    fd_r_deriv_ = std::make_unique<FilterDiff>(filter_omega_r, filter_zeta_r);

    // Setup Sync Write object for goal current
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    RCLCPP_INFO(this->get_logger(), "DroneDynamixelBridgeNode initialized.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down DroneDynamixelBridgeNode...");
    // Disable torque on all Dynamixels before closing the port.
    for (uint8_t id : DXL_IDS) {
      if (portHandler_ && portHandler_->is_open() && packetHandler_) {
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, nullptr);
        std::this_thread::sleep_for(10ms);
      }
    }
    if (portHandler_ && portHandler_->is_open()) {
      portHandler_->closePort();
      RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");
    }
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
  // Subscribers and timer
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex odom_mutex_, ang_vel_mutex_;
  px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
  px4_msgs::msg::VehicleAngularVelocity::SharedPtr latest_angular_velocity_msg_;
  rclcpp::Time prev_time_;
  bool odometry_received_ = false, angular_velocity_received_ = false, first_run_ = true;
  
  // Kinematics variables
  double phi_raw_ = 0, theta_raw_ = 0, psi_raw_ = 0;
  double p_raw_ = 0, q_raw_ = 0, r_raw_ = 0;
  double p_dot_filt_ = 0, q_dot_filt_ = 0, r_dot_filt_ = 0;
  double phi_dot_raw_calc_ = 0, theta_dot_raw_calc_ = 0, psi_dot_raw_calc_ = 0;
  
  // Filter objects (for differentiating body rates)
  std::unique_ptr<FilterDiff> fd_p_deriv_, fd_q_deriv_, fd_r_deriv_;
  
  // Dynamixel SDK objects
  dynamixel::PortHandler *portHandler_ = nullptr;
  dynamixel::PacketHandler *packetHandler_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;
  
  // Dynamics and inertia
  std::shared_ptr<EqDynamics> dynamics_;
  const Eigen::Matrix3d I_CUAV_, I_CRB_, I_CPR_, I_CYR_;
  Eigen::Matrix3d C_J_LRB_ = Eigen::Matrix3d::Identity(), C_J_LPR_, C_J_LYR_, C_J_I_;
  
  // Current time variable
  rclcpp::Time current_time_;
  
  // Callback: Odometry subscription
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odometry_msg_ = msg;
    odometry_received_ = true;
  }
  
  // Callback: Angular velocity subscription (expected order: [p, q, r])
  void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ang_vel_mutex_);
    latest_angular_velocity_msg_ = msg;
    p_raw_ = msg->xyz[0];
    q_raw_ = msg->xyz[1];
    r_raw_ = msg->xyz[2];
    angular_velocity_received_ = true;
  }
  
  // Timer callback (runs at 200Hz)
  void timer_callback() {
    current_time_ = this->now();
    double dt = (current_time_ - prev_time_).seconds();
    if (dt <= 1e-6 || dt > 0.1) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid dt: %.4f s", dt);
      return;
    }
    prev_time_ = current_time_;
    
    px4_msgs::msg::VehicleOdometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odometry_received_) {
        RCLCPP_WARN(this->get_logger(), "No odometry received yet.");
        return;
      }
      odom = latest_odometry_msg_;
    }
    {
      std::lock_guard<std::mutex> lock(ang_vel_mutex_);
      if (!angular_velocity_received_) {
        RCLCPP_WARN(this->get_logger(), "No angular velocity received yet.");
        return;
      }
    }
    
    // Get rotation from odometry (PX4 uses [w, x, y, z])
    Eigen::Quaterniond q(odom->q[0], odom->q[1], odom->q[2], odom->q[3]);
    q.normalize();
    // Get rotation matrix from inertial to body; we then use its transpose to convert body to inertial if needed.
    C_J_I_ = q.toRotationMatrix().transpose();
    
    // Extract raw Euler angles using a ZYX convention
    extractEulerAnglesZYX(C_J_I_, phi_raw_, theta_raw_, psi_raw_);
    
    // Update body rate filters (for filtered angular acceleration)
    if (first_run_) {
      fd_p_deriv_->reset(p_raw_);
      fd_q_deriv_->reset(q_raw_);
      fd_r_deriv_->reset(r_raw_);
      first_run_ = false;
      RCLCPP_INFO(this->get_logger(), "First run: Filters initialized.");
      return;
    }
    fd_p_deriv_->update(p_raw_, dt);
    fd_q_deriv_->update(q_raw_, dt);
    fd_r_deriv_->update(r_raw_, dt);
    p_dot_filt_ = fd_p_deriv_->getFilteredDerivative();
    q_dot_filt_ = fd_q_deriv_->getFilteredDerivative();
    r_dot_filt_ = fd_r_deriv_->getFilteredDerivative();
    Eigen::Vector3d omega_raw(p_raw_, q_raw_, r_raw_);
    Eigen::Vector3d alpha_filt(p_dot_filt_, q_dot_filt_, r_dot_filt_);
    
    // Calculate Euler rates from raw body rates and Euler angles
    calculateEulerRates(phi_raw_, theta_raw_, p_raw_, q_raw_, r_raw_,
                        phi_dot_raw_calc_, theta_dot_raw_calc_, psi_dot_raw_calc_);
    
    // Update additional DCMs based on raw angles (for control allocation)
    update_dcms(phi_raw_, theta_raw_, psi_raw_);
    
    // --- Compute torques for each component (using simplified mixed method) ---
    // For now, if no extra transformation is available, assume the transformation is identity (UAV frame)
    Eigen::Matrix3d C_LUAV = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_UAV = dynamics_->compute_torque_simplified_mixed(I_CUAV_, C_LUAV.transpose(), omega_raw, alpha_filt);
    Eigen::Vector3d T_RB  = dynamics_->compute_torque_simplified_mixed(I_CRB_, C_J_LRB_, omega_raw, alpha_filt);
    Eigen::Vector3d T_PR  = dynamics_->compute_torque_simplified_mixed(I_CPR_, C_J_LPR_, omega_raw, alpha_filt);
    Eigen::Vector3d T_YR  = dynamics_->compute_torque_simplified_mixed(I_CYR_, C_J_LYR_, omega_raw, alpha_filt);
    
    Eigen::Vector3d total_inertial_torque = T_UAV + T_RB + T_PR + T_YR;
    Eigen::Vector3d required_motor_torque = -total_inertial_torque; // Negate to cancel inertial effects
    
    // Compute inverse Jacobian from raw phi and theta (for control allocation to gimbal motors)
    Eigen::Matrix3d Gamma_inv = compute_inverse_jacobian(phi_raw_, theta_raw_);
    Eigen::Vector3d gimbal_torques = Gamma_inv * required_motor_torque;
    double roll_torque = gimbal_torques.x(), pitch_torque = gimbal_torques.y(), yaw_torque = gimbal_torques.z();
    
    // Convert calculated torques into goal current steps for each Dynamixel (using calibrated fits)
    int32_t roll_curr = torque_to_goal_current_calibrated(roll_torque);
    int32_t pitch_curr = torque_to_goal_current_calibrated(pitch_torque);
    int32_t yaw_curr = torque_to_goal_current_calibrated(yaw_torque);
    
    if (!send_dynamixel_commands(roll_curr, pitch_curr, yaw_curr)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send Dynamixel commands.");
    }
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
      "Angles: [%.2f, %.2f, %.2f] | Rates: [%.2f, %.2f, %.2f] | Filt Accel: [%.2f, %.2f, %.2f] | Euler Rates: [%.2f, %.2f] | Torques: [%.3f, %.3f, %.3f] | Currents: [%d, %d, %d]",
      phi_raw_, theta_raw_, psi_raw_,
      p_raw_, q_raw_, r_raw_,
      p_dot_filt_, q_dot_filt_, r_dot_filt_,
      phi_dot_raw_calc_, theta_dot_raw_calc_,
      roll_torque, pitch_torque, yaw_torque,
      roll_curr, pitch_curr, yaw_curr);
  }
  
  // Helper: extract Euler angles from a rotation matrix (assumes ZYX rotation sequence)
  void extractEulerAnglesZYX(const Eigen::Matrix3d& R, double& phi, double& theta, double& psi) {
    theta = -asin(R(0,2)); // R(0,2) = -sin(theta)
    if (std::abs(cos(theta)) > 1e-6) {
      phi = atan2(R(1,2), R(2,2));
      psi = atan2(R(0,1), R(0,0));
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock detected!");
      phi = 0.0;
      psi = atan2(R(1,0), R(1,1));
    }
  }
  
  // Helper: calculate Euler rates from body rates (p,q,r) and Euler angles (ZYX)
  void calculateEulerRates(double phi, double theta, double p, double q, double r,
                           double& phi_dot, double& theta_dot, double& psi_dot) {
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta);
    if (std::abs(cth) < 1e-6) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock in Euler rate calculation");
      phi_dot = p;
      theta_dot = q * cphi - r * sphi;
      psi_dot = 0.0;
    } else {
      double tan_th = tan(theta);
      phi_dot = p + q * sphi * tan_th + r * cphi * tan_th;
      theta_dot = q * cphi - r * sphi;
      psi_dot = q * sphi / cth + r * cphi / cth;
    }
  }
  
  // Helper: update DCMs based on raw Euler angles. Here we update only two DCMs (for pitch ring and yaw ring)
  void update_dcms(double phi, double theta, double /*psi*/) {
    double cphi = cos(phi), sphi = sin(phi), cth = cos(theta), sth = sin(theta);
    // C_J/LPR: rotation about X-axis by phi
    C_J_LPR_ << 1, 0, 0,
                0, cphi, sphi,
                0, -sphi, cphi;
    // C_J/LYR: using a formula from PDF Eq 18b (verify with your conventions)
    C_J_LYR_ << cth, sphi * sth, cphi * sth,
                0,      cphi,     -sphi,
                -sth, sphi * cth, cphi * cth;
  }
  
  // Helper: compute inverse Jacobian for gimbal control based on phi and theta (assuming a ZYX decomposition)
  Eigen::Matrix3d compute_inverse_jacobian(double phi, double theta) {
    double cphi = cos(phi), sphi = sin(phi), cth = cos(theta);
    Eigen::Matrix3d G_inv = Eigen::Matrix3d::Identity();
    if (std::abs(cth) < 1e-6) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock! Using identity inverse Jacobian.");
    } else {
      double tan_th = tan(theta);
      G_inv << 1, sphi * tan_th, cphi * tan_th,
               0, cphi,         -sphi,
               0, sphi / cth,    cphi / cth;
    }
    return G_inv;
  }
  
  // Helper: convert torque [Nm] to Dynamixel goal current steps using calibrated fits
  int32_t torque_to_goal_current_calibrated(double torque_nm) {
    double current_A;
    if (torque_nm >= 0) {
      current_A = 0.6370 * torque_nm + 0.0395; // Positive Torque equation
    } else {
      current_A = 0.6439 * torque_nm - 0.1097; // Negative Torque equation
    }
    double steps = current_A / CURRENT_STEP;
    double max_steps = DEFAULT_CURRENT_LIMIT_STEPS;
    steps = std::max(-max_steps, std::min(max_steps, steps));
    return static_cast<int32_t>(std::round(steps));
  }
  
  // Helper: write one byte to Dynamixel with logging using the SDK
  void write_byte_with_log(uint8_t id, uint16_t addr, uint8_t val, const std::string &op) {
    if (!portHandler_ || !packetHandler_)
      return;
    uint8_t err = 0;
    int res = packetHandler_->write1ByteTxRx(portHandler_, id, addr, val, &err);
    if (res != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "ID:%d Addr:%d (%s) Comm Fail: %s", id, addr, op.c_str(), packetHandler_->getTxRxResult(res));
    } else if (err != 0) {
      RCLCPP_ERROR(this->get_logger(), "ID:%d Addr:%d (%s) DXL Error: %s", id, addr, op.c_str(), packetHandler_->getRxPacketError(err));
    }
  }
  
  // Helper: send current commands to Dynamixels using SyncWrite
  bool send_dynamixel_commands(int32_t current_roll, int32_t current_pitch, int32_t current_yaw) {
    if (!groupSyncWrite_ || !packetHandler_)
      return false;
    bool ok = true;
    uint8_t p[LEN_GOAL_CURRENT];
    // Roll
    uint16_t w_r = static_cast<uint16_t>(current_roll);
    p[0] = DXL_LOBYTE(w_r);
    p[1] = DXL_HIBYTE(w_r);
    if (!groupSyncWrite_->addParam(DXL_ID_ROLL, p))
      ok = false;
    // Pitch
    uint16_t w_p = static_cast<uint16_t>(current_pitch);
    p[0] = DXL_LOBYTE(w_p);
    p[1] = DXL_HIBYTE(w_p);
    if (!groupSyncWrite_->addParam(DXL_ID_PITCH, p))
      ok = false;
    // Yaw
    uint16_t w_y = static_cast<uint16_t>(current_yaw);
    p[0] = DXL_LOBYTE(w_y);
    p[1] = DXL_HIBYTE(w_y);
    if (!groupSyncWrite_->addParam(DXL_ID_YAW, p))
      ok = false;
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed addParam SyncWrite.");
      groupSyncWrite_->clearParam();
      return false;
    }
    int res = groupSyncWrite_->txPacket();
    bool tx_ok = (res == COMM_SUCCESS);
    if (!tx_ok) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "SyncWrite fail: %s", packetHandler_->getTxRxResult(res));
    }
    groupSyncWrite_->clearParam();
    return tx_ok;
  }
  
  // Helper: Dynamixel initialization using SDK
  bool initialize_dynamixels() {
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!portHandler_ || !packetHandler_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get SDK handlers.");
      return false;
    }
    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICENAME);
      return false;
    }
    if (!portHandler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE);
      portHandler_->closePort();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Opened port %s @ %d baud", DEVICENAME, BAUDRATE);
    uint8_t op_mode = 0; // Set to Current Control mode
    for (uint8_t id : DXL_IDS) {
      write_byte_with_log(id, ADDR_OPERATING_MODE, op_mode, "Set OpMode");
      std::this_thread::sleep_for(30ms);
    }
    for (uint8_t id : DXL_IDS) {
      write_byte_with_log(id, ADDR_TORQUE_ENABLE, 1, "Enable Torque");
    }
    RCLCPP_INFO(this->get_logger(), "Dynamixels initialized (Current Control, Torque Enabled).");
    return true;
  }
  
    // Variable to hold the rotation matrix from Inertial to Body frame (from odometry)
    Eigen::Matrix3d C_J_I_;
  
}; // End class DroneDynamixelBridgeNode

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DroneDynamixelBridgeNode> node;
  try {
      node = std::make_shared<DroneDynamixelBridgeNode>();
      RCLCPP_INFO(node->get_logger(), "Node created successfully. Spinning...");
      rclcpp::spin(node);
  } catch (const std::runtime_error &e) {
      RCLCPP_FATAL(rclcpp::get_logger("Main"), "Node initialization failed: %s", e.what());
  } catch (const std::exception &e) {
      RCLCPP_FATAL(rclcpp::get_logger("Main"), "An unexpected error occurred: %s", e.what());
  }
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Shutting down node.");
  rclcpp::shutdown();
  return 0;
}
