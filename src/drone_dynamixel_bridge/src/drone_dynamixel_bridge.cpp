 #include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
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

using namespace std::chrono_literals;

const uint8_t DXL_ID_ROLL = 2;
const uint8_t DXL_ID_PITCH = 1;
const uint8_t DXL_ID_YAW = 3;
const std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
const char* DEVICENAME = "/dev/ttyUSB1"; // <<< adjust as necessary
const int BAUDRATE = 4000000;
const float PROTOCOL_VERSION = 2.0;
const uint16_t ADDR_OPERATING_MODE = 11, ADDR_TORQUE_ENABLE = 64, ADDR_GOAL_CURRENT = 102;
const uint16_t LEN_OPERATING_MODE = 1, LEN_TORQUE_ENABLE = 1, LEN_GOAL_CURRENT = 2;

static const double CURRENT_STEP = 0.0045; 
static const double DEFAULT_CURRENT_LIMIT_STEPS = 1800.0; 


class FilterDiff {
public:
    FilterDiff(double wn, double zeta) 
      : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), initialized_(false) {}

    void update(double input, double dt) {
        if (dt <= 1e-9) { return; }
        if (!initialized_) {
            x1_ = input;
            x2_ = 0.0;
            initialized_ = true;
        } else {
            double x1_dot = x2_;
            double x2_dot = -wn_*wn_*x1_ - 2.0*zeta_*wn_*x2_ + wn_*wn_*input;
            x1_ += x1_dot * dt;
            x2_ += x2_dot * dt;
        }
    }

    double getFilteredDerivative() const { return x2_; }

    void reset(double initial_value = 0.0) {
        x1_ = initial_value;
        x2_ = 0.0;
        initialized_ = true;
    }

    void resetState(double val, double deriv) {
        x1_ = val;
        x2_ = deriv;
        initialized_ = true;
    }

private:
    double wn_;
    double zeta_;
    double x1_;
    double x2_;
    bool initialized_;
};

// --- Dynamics Equations Class ---
class EqDynamics {
public:
    EqDynamics() {}

    Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) const {
        Eigen::Matrix3d m;
        m <<      0, -v.z(),  v.y(),
               v.z(),     0, -v.x(),
              -v.y(),  v.x(),     0;
        return m;
    }

    Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &I,
                                     const Eigen::Vector3d &omega_dot) const {
        return I * omega_dot;
    }

    Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &I,
                                            const Eigen::Vector3d &omega1,
                                            const Eigen::Vector3d &omega2) const {
        return omega1.cross(I * omega2);
    }

    // Simplified torque calculation
    Eigen::Vector3d compute_torque_simplified_mixed(
        const Eigen::Matrix3d &ILi_inLi,
        const Eigen::Matrix3d &CJLi,
        const Eigen::Vector3d &omega_LiI_inLi_raw,
        const Eigen::Vector3d &omega_dot_LiI_inLi_filt) const
    {
        Eigen::Vector3d term1_inLi = computeIDomegaDt(ILi_inLi, omega_dot_LiI_inLi_filt);
        Eigen::Vector3d term2_inLi = computeOmegaCrossIomega(ILi_inLi,
                                                             omega_LiI_inLi_raw,
                                                             omega_LiI_inLi_raw);
        return CJLi * (term1_inLi + term2_inLi);
    }
};

// --- ROS2 Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode() 
    : Node("drone_dynamixel_bridge_node"),
      // Sample inertia matrices, adapt them as needed
      I_CUAV_((Eigen::Matrix3d() <<  0.010679,-0.000001,-0.000072,
                                   -0.000001, 0.017318, 0.000007,
                                   -0.000072, 0.000007, 0.026610).finished()),
      I_CRB_((Eigen::Matrix3d() <<   0.000847,-0.000004,-0.000010,
                                   -0.000004, 0.098249, 0.000000,
                                   -0.000010, 0.000000, 0.098096).finished()),
      I_CPR_((Eigen::Matrix3d() <<   0.640302, 0.000043, 0.000078,
                                    0.000043, 0.570491,-0.000025,
                                    0.000078,-0.000025, 1.200050).finished()),
      I_CYR_((Eigen::Matrix3d() <<   2.384163,-0.000049,-0.000104,
                                   -0.000049, 1.110062, 0.000781,
                                   -0.000104, 0.000781, 1.286313).finished())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Node...");

    // Filter parameters
    double filter_omega_p_deriv = 70.0;
    double filter_zeta_p_deriv  = 0.7;
    double filter_omega_q_deriv = 70.0;
    double filter_zeta_q_deriv  = 0.7;
    double filter_omega_r_deriv = 70.0;
    double filter_zeta_r_deriv  = 0.7;

    // Subscriptions
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1));

    angular_velocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", qos,
        std::bind(&DroneDynamixelBridgeNode::angular_velocity_callback, this, std::placeholders::_1));

    // Timer & SDK
    timer_ = this->create_wall_timer(
        5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this)
    );
    if (!initialize_dynamixels()) {
      throw std::runtime_error("Dynamixel initialization failed");
    }

    dynamics_ = std::make_shared<EqDynamics>();

    // Initialize filters
    fd_p_deriv_ = std::make_unique<FilterDiff>(filter_omega_p_deriv, filter_zeta_p_deriv);
    fd_q_deriv_ = std::make_unique<FilterDiff>(filter_omega_q_deriv, filter_zeta_q_deriv);
    fd_r_deriv_ = std::make_unique<FilterDiff>(filter_omega_r_deriv, filter_zeta_r_deriv);

    // Prepare for sync writes
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(
        portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    prev_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");

    // Disable torque on each servo
    for (uint8_t id : DXL_IDS) {
      if (portHandler_ && portHandler_->is_open() && packetHandler_) {
        packetHandler_->write1ByteTxRx(
            portHandler_, id, ADDR_TORQUE_ENABLE, 0, nullptr);
        rclcpp::sleep_for(10ms);
      }
    }

    // Close port
    if (portHandler_ && portHandler_->is_open()) {
      portHandler_->closePort();
      RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");
    }
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
    // Subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Mutex & message data
    std::mutex odom_mutex_, ang_vel_mutex_;
    px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
    px4_msgs::msg::VehicleAngularVelocity::SharedPtr latest_angular_velocity_msg_;

    // Time
    rclcpp::Time prev_time_;

    // State flags
    bool odometry_received_ = false;
    bool angular_velocity_received_ = false;
    bool first_run_ = true;

    // Kinematics (Raw Angles/Vel, Filtered Accel)
    double phi_raw_=0, theta_raw_=0, psi_raw_=0;                   
    double p_raw_=0, q_raw_=0, r_raw_=0;                           
    double p_dot_filt_=0, q_dot_filt_=0, r_dot_filt_=0;            
    double phi_dot_raw_calc_=0, theta_dot_raw_calc_=0, psi_dot_raw_calc_=0;

    // Filter Objects
    std::unique_ptr<FilterDiff> fd_p_deriv_, fd_q_deriv_, fd_r_deriv_;

    // SDK / Dynamics
    dynamixel::PortHandler *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;
    std::shared_ptr<EqDynamics> dynamics_;

    // Inertias
    const Eigen::Matrix3d I_CUAV_;
    const Eigen::Matrix3d I_CRB_;
    const Eigen::Matrix3d I_CPR_;
    const Eigen::Matrix3d I_CYR_;

    // Example DCM placeholders
    // In real usage, you might keep them updated from angles or quaternions, etc.
    Eigen::Matrix3d C_J_LRB_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_J_LPR_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_J_LYR_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_J_I_   = Eigen::Matrix3d::Identity();

    // --- Callbacks ---
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_odometry_msg_ = msg;
      odometry_received_ = true;
    }

    void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(ang_vel_mutex_);
      latest_angular_velocity_msg_ = msg;

      // Store raw body rates
      p_raw_ = msg->xyz[0];
      q_raw_ = msg->xyz[1];
      r_raw_ = msg->xyz[2];
      angular_velocity_received_ = true;
    }

    void timer_callback() {
      // Basic timekeeping
      rclcpp::Time current_time = this->now();
      double dt = (current_time - prev_time_).seconds();
      if (dt <= 1e-6 || dt > 0.1) {
        // Skip first or too large intervals
        prev_time_ = current_time;
        return;
      }
      prev_time_ = current_time;

      // Check for messages
      px4_msgs::msg::VehicleOdometry::SharedPtr odom;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if(!odometry_received_) return;
        odom = latest_odometry_msg_;
      }
      {
        std::lock_guard<std::mutex> lock(ang_vel_mutex_);
        if(!angular_velocity_received_) return;
      }

      // Convert quaternion to rotation matrix
      Eigen::Quaterniond q_I_J(odom->q[0], odom->q[1], odom->q[2], odom->q[3]);
      q_I_J.normalize();
      // For local usage, C_J_I_ = R^T, so that's fine:
      C_J_I_ = q_I_J.toRotationMatrix().transpose();

      // Extract raw Euler angles from matrix
      extractEulerAnglesZYX(C_J_I_, phi_raw_, theta_raw_, psi_raw_);

      // Filtered body acceleration
      if (first_run_) {
        fd_p_deriv_->reset(p_raw_);
        fd_q_deriv_->reset(q_raw_);
        fd_r_deriv_->reset(r_raw_);
        first_run_ = false;
        return; // skip further processing on the very first run
      }
      fd_p_deriv_->update(p_raw_, dt);
      fd_q_deriv_->update(q_raw_, dt);
      fd_r_deriv_->update(r_raw_, dt);
      p_dot_filt_ = fd_p_deriv_->getFilteredDerivative();
      q_dot_filt_ = fd_q_deriv_->getFilteredDerivative();
      r_dot_filt_ = fd_r_deriv_->getFilteredDerivative();

      // Euler rates from p,q,r
      calculateEulerRates(phi_raw_, theta_raw_, p_raw_, q_raw_, r_raw_,
                          phi_dot_raw_calc_, theta_dot_raw_calc_, psi_dot_raw_calc_);

      // Update DCM placeholders from angles
      update_dcms(phi_raw_, theta_raw_, psi_raw_);

      // For demonstration, assume local relative accelerations are zero
      // so we only have absolute alpha from the filter, etc.
      // Then compute inertial torques in each local frame
      Eigen::Vector3d omega_JI_raw(p_raw_, q_raw_, r_raw_);
      Eigen::Vector3d omega_dot_JI_filt(p_dot_filt_, q_dot_filt_, r_dot_filt_);

      // Example: just compute partial torques from each inertia
      Eigen::Vector3d T_UAV = dynamics_->compute_torque_simplified_mixed(
          I_CUAV_, Eigen::Matrix3d::Identity(),
          omega_JI_raw, omega_dot_JI_filt);
      Eigen::Vector3d T_RB  = dynamics_->compute_torque_simplified_mixed(
          I_CRB_, C_J_LRB_,
          omega_JI_raw, omega_dot_JI_filt);
      Eigen::Vector3d T_PR  = dynamics_->compute_torque_simplified_mixed(
          I_CPR_, C_J_LPR_,
          omega_JI_raw, omega_dot_JI_filt);
      Eigen::Vector3d T_YR  = dynamics_->compute_torque_simplified_mixed(
          I_CYR_, C_J_LYR_,
          omega_JI_raw, omega_dot_JI_filt);

      // Sum torques
      Eigen::Vector3d total_inertial_torque = T_UAV + T_RB + T_PR + T_YR;
      // Suppose we want negative to hold the body still:
      Eigen::Vector3d required_motor_torque = -total_inertial_torque;

      // Example: map required torque to gimbal axis torque
      Eigen::Matrix3d Gamma_inv = compute_inverse_jacobian(phi_raw_, theta_raw_);
      Eigen::Vector3d gimbal_torques = Gamma_inv * required_motor_torque;

      double roll_torque  = gimbal_torques.x();
      double pitch_torque = gimbal_torques.y();
      double yaw_torque   = gimbal_torques.z();

      int32_t roll_curr  = torque_to_goal_current_calibrated(roll_torque);
      int32_t pitch_curr = torque_to_goal_current_calibrated(pitch_torque);
      int32_t yaw_curr   = torque_to_goal_current_calibrated(yaw_torque);

      // Send commands to dynamixel servos
      if(!send_dynamixel_commands(roll_curr, pitch_curr, yaw_curr)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send Dynamixel commands");
      }
    }

    // --- Helper function to initialize Dynamixels ---
    bool initialize_dynamixels() {
      // Create handlers
      portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
      packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

      // Try opening port
      if (!portHandler_->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Dynamixel port!");
        return false;
      }
      // Set baud rate
      if (!portHandler_->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Dynamixel baud rate!");
        return false;
      }

      // Example: set each servo's mode & enable torque
      for(uint8_t id : DXL_IDS) {
        // Operating mode: e.g. current control = 0 for X-series. Check your device’s manual.
        uint8_t desired_mode = 0; 
        write_byte_with_log(id, ADDR_OPERATING_MODE, desired_mode, "Set OperatingMode");

        // Torque enable
        write_byte_with_log(id, ADDR_TORQUE_ENABLE, 1, "TorqueEnable");
      }

      RCLCPP_INFO(this->get_logger(), "Dynamixels initialized successfully.");
      return true;
    }

    // --- Extract Euler angles (ZYX convention) from a rotation matrix ---
    void extractEulerAnglesZYX(const Eigen::Matrix3d& R,
                               double& phi, double& theta, double& psi)
    {
      psi   = std::atan2(R(1,0), R(0,0));
      theta = -std::asin(R(2,0));
      phi   = std::atan2(R(2,1), R(2,2));
    }

    // --- Convert body rates p, q, r into Euler angle rates (phi_dot, theta_dot, psi_dot) ---
    void calculateEulerRates(double phi, double theta,
                             double p, double q, double r,
                             double& phi_dot, double& theta_dot, double& psi_dot)
    {
      double cosTheta = std::cos(theta);
      double cosPhi = std::cos(phi);
      double sinPhi = std::sin(phi);
      double tanTheta = std::tan(theta);

      // Protect from div by zero if cos(theta) ~ 0
      if (std::fabs(cosTheta) < 1e-8) {
        cosTheta = (cosTheta >= 0.0) ? 1e-8 : -1e-8;
      }

      phi_dot   = p + sinPhi*tanTheta*q + cosPhi*tanTheta*r;
      theta_dot =           cosPhi*q - sinPhi*r;
      psi_dot   =     sinPhi/cosTheta*q + cosPhi/cosTheta*r;
    }

    // --- Simple update of local DCM placeholders from raw angles
    //     In real usage, you’d do your actual transformations here.
    void update_dcms(double phi, double theta, double psi) {
      Eigen::AngleAxisd Rx(phi,   Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd Ry(theta, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd Rz(psi,   Eigen::Vector3d::UnitZ());

      // Example placeholders
      C_J_LRB_ = Rx.toRotationMatrix();
      C_J_LPR_ = Ry.toRotationMatrix();
      C_J_LYR_ = Rz.toRotationMatrix();
    }

    // --- Inverse of some Jacobian that maps motor torques to 3D torque.
    //     Adjust for your actual gimbal geometry.
    Eigen::Matrix3d compute_inverse_jacobian(double /*phi*/, double /*theta*/) {
      return Eigen::Matrix3d::Identity(); // placeholder
    }

    // --- Your piecewise calibration from torque (Nm) to servo current steps
    int32_t torque_to_goal_current_calibrated(double torque_nm) {
      // 1) Convert torque to current (A), piecewise:
      double current_amps;
      if (torque_nm >= 0.0) {
        
        current_amps = 0.6370 * torque_nm + 0.0395;
      } else {
        current_amps = 0.6439 * torque_nm - 0.1097;
      }

      // 2) Convert current (A) to “goal current” steps 
      //    For many Dynamixel X-series, step ~ 0.00269 A
      double steps_float = current_amps / CURRENT_STEP;

      // 3) Clamp to avoid exceeding servo rating
      if (steps_float >  DEFAULT_CURRENT_LIMIT_STEPS)  steps_float =  DEFAULT_CURRENT_LIMIT_STEPS;
      if (steps_float < -DEFAULT_CURRENT_LIMIT_STEPS)  steps_float = -DEFAULT_CURRENT_LIMIT_STEPS;

      // 4) Round and return
      return static_cast<int32_t>(std::round(steps_float));
    }

    // --- Helper to write a single byte (with logging) ---
    void write_byte_with_log(uint8_t id, uint16_t addr, uint8_t val, const std::string& op) {
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, val, nullptr);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed %s (ID %d): %s",
                     op.c_str(), id, packetHandler_->getTxRxResult(dxl_comm_result));
      } else {
        RCLCPP_INFO(this->get_logger(), "%s (ID %d) -> 0x%02X success", 
                    op.c_str(), id, (unsigned)val);
      }
    }

    // --- Send the current commands to the three Dynamixel servos ---
    bool send_dynamixel_commands(int32_t current_roll,
                                 int32_t current_pitch,
                                 int32_t current_yaw)
    {
      // Prepare syncwrite for all 3 motors
      groupSyncWrite_->clearParam();

      // roll
      {
        uint8_t param_goal_current[2];
        param_goal_current[0] = static_cast<uint8_t>(current_roll & 0xFF);
        param_goal_current[1] = static_cast<uint8_t>((current_roll >> 8) & 0xFF);
        if (!groupSyncWrite_->addParam(DXL_ID_ROLL, param_goal_current)) {
          return false;
        }
      }
      // pitch
      {
        uint8_t param_goal_current[2];
        param_goal_current[0] = static_cast<uint8_t>(current_pitch & 0xFF);
        param_goal_current[1] = static_cast<uint8_t>((current_pitch >> 8) & 0xFF);
        if (!groupSyncWrite_->addParam(DXL_ID_PITCH, param_goal_current)) {
          return false;
        }
      }
      // yaw
      {
        uint8_t param_goal_current[2];
        param_goal_current[0] = static_cast<uint8_t>(current_yaw & 0xFF);
        param_goal_current[1] = static_cast<uint8_t>((current_yaw >> 8) & 0xFF);
        if (!groupSyncWrite_->addParam(DXL_ID_YAW, param_goal_current)) {
          return false;
        }
      }

      // Transmit
      int dxl_comm_result = groupSyncWrite_->txPacket();
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), 
                     "GroupSyncWrite failed: %s", 
                     packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      return true;
    }
};

// --- Main Function ---
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DroneDynamixelBridgeNode> node = nullptr;
  try {
      node = std::make_shared<DroneDynamixelBridgeNode>();
      RCLCPP_INFO(node->get_logger(), "Node created successfully. Spinning...");
      rclcpp::spin(node); // Runs callbacks until shutdown
  } catch (const std::runtime_error & e) {
      RCLCPP_FATAL(rclcpp::get_logger("Main"), "Node initialization failed: %s", e.what());
  } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("Main"), "An unexpected error occurred: %s", e.what());
  }
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Shutting down node.");
  rclcpp::shutdown();
  return 0;
}
