#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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
#include <thread> // For std::this_thread::sleep_for
#include <fstream> // For file output
#include <iomanip> // For std::fixed, std::setprecision

using namespace std::chrono_literals;

// --- Dynamixel Configuration (Write-only) ---
const uint8_t DXL_ID_ROLL = 2;
const uint8_t DXL_ID_PITCH = 1;
const uint8_t DXL_ID_YAW = 3;
const std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
const char* DEVICENAME = "/dev/ttyUSB1"; // <<< VERIFY this port name
const int BAUDRATE = 4000000;
const float PROTOCOL_VERSION = 2.0;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_GOAL_CURRENT = 102;
const uint16_t LEN_TORQUE_ENABLE = 1;
const uint16_t LEN_GOAL_CURRENT = 2;
const double CURRENT_LSB         = 0.0045;       // 4.5 mA per tick
const int    CURRENT_OFFSET_RAW  = 2048;         // Register value at 0 A
const int    CURRENT_RAW_MIN     = 0;            // Min register (0 A => RAW=2048)
const int    CURRENT_RAW_MAX     = 4095;         // Max register (±9.2115 A)

// --- Filter Differentiator Class ---
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
// Making accessible for potential resetState if needed later, no harm
  double wn_, zeta_, x1_, x2_;
  bool initialized_;
};

// --- Dynamics Equations Class ---
class EqDynamics {
public:
  EqDynamics() {}
  Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) const {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
  }
  Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &I, const Eigen::Vector3d &omega_dot) const {
    return I * omega_dot;
  }
  Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &I,
                                          const Eigen::Vector3d &omega1,
                                          const Eigen::Vector3d &omega2) const {
    return omega1.cross(I * omega2);
  }
  Eigen::Vector3d compute_torque_simplified_mixed(
      const Eigen::Matrix3d &ILi_inLi, const Eigen::Matrix3d &CJLi,
      const Eigen::Vector3d &omega_LiI_inLi_raw,
      const Eigen::Vector3d &omega_dot_LiI_inLi_filt) const {
    Eigen::Vector3d term1 = computeIDomegaDt(ILi_inLi, omega_dot_LiI_inLi_filt);
    Eigen::Vector3d term2 = computeOmegaCrossIomega(ILi_inLi, omega_LiI_inLi_raw, omega_LiI_inLi_raw);
    return CJLi * (term1 + term2);
  }
};

// --- ROS2 Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode()
    : Node("drone_dynamixel_bridge_node"),
      // Inertia tensors
      I_CUAV_((Eigen::Matrix3d() <<  0.012576, -0.000001, -0.000069, -0.000001, 0.019215,  0.000012, -0.000069, 0.000012,  0.026610).finished()),
      I_CRB_((Eigen::Matrix3d() <<   0.000866, -0.000001, -0.000068, -0.000001, 0.101389,  0.000004, -0.000068, 0.000004,  0.101225).finished()),
      I_CPR_((Eigen::Matrix3d() <<   0.640302,  0.000043,  0.000078,  0.000043,  0.570491, -0.000025,  0.000078, -0.000025,  1.200050).finished()),
      I_CYR_((Eigen::Matrix3d() <<   2.384163, -0.000049, -0.000104, -0.000049, 1.110062,  0.000781, -0.000104, 0.000781,  1.286313).finished()),
      C_J_LRB_(Eigen::Matrix3d::Identity())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Node...");

    // --- CSV File Setup ---
    log_file_name_ = "diagnostic_log.csv";
    log_file_.open(log_file_name_, std::ios::out | std::ios::trunc); // Open in write mode, truncate if exists
    if (log_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Opened diagnostic log file: %s", log_file_name_.c_str());
        // Write CSV Header
        log_file_ << "Timestamp_s,"
                  << "p_raw_rads," << "q_raw_rads," << "r_raw_rads,"
                  << "p_dot_filt_rads2," << "q_dot_filt_rads2," << "r_dot_filt_rads2,"
                  << "TotalInertialTorqueX_Nm," << "TotalInertialTorqueY_Nm," << "TotalInertialTorqueZ_Nm,"
                  << "GimbalTorqueRoll_Nm," << "GimbalTorquePitch_Nm," << "GimbalTorqueYaw_Nm,"
                  << "FinalTorqueCmdRoll_Nm," << "FinalTorqueCmdPitch_Nm," << "FinalTorqueCmdYaw_Nm,"
                  << "DXLCurrentCmdRoll_raw," << "DXLCurrentCmdPitch_raw," << "DXLCurrentCmdYaw_raw"
                  << std::endl; // Use std::endl to ensure header is flushed
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open diagnostic log file: %s", log_file_name_.c_str());
    }

    // Filter parameters
    double filter_omega_p_deriv = 70.0; double filter_zeta_p_deriv = 0.7;
    double filter_omega_q_deriv = 70.0; double filter_zeta_q_deriv = 0.7;
    double filter_omega_r_deriv = 70.0; double filter_zeta_r_deriv = 0.7;

    // Subscriptions
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1));

    // Timer & SDK Init
    timer_ = this->create_wall_timer(5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this)); // 200 Hz
    prev_time_ = this->now();

    if (!initialize_dynamixels()) {
      throw std::runtime_error("Dynamixel initialization failed");
    }
    dynamics_ = std::make_shared<EqDynamics>();

    // Initialize Filters
    fd_p_deriv_ = std::make_unique<FilterDiff>(filter_omega_p_deriv, filter_zeta_p_deriv);
    fd_q_deriv_ = std::make_unique<FilterDiff>(filter_omega_q_deriv, filter_zeta_q_deriv);
    fd_r_deriv_ = std::make_unique<FilterDiff>(filter_omega_r_deriv, filter_zeta_r_deriv);

    // Initialize Sync Write object
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    RCLCPP_INFO(this->get_logger(), "Node initialized.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down DroneDynamixelBridgeNode...");
    // Disable torque on all Dynamixels
    for (uint8_t id : DXL_IDS) {
      if (portHandler_ && packetHandler_) {
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, nullptr);
        std::this_thread::sleep_for(10ms);
      }
    }
    // Close port if handler is valid
    if (portHandler_) {
      portHandler_->closePort();
      RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");
    }
    // Close log file
    if (log_file_.is_open()) {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Closed diagnostic log file: %s", log_file_name_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
  // --- Subscriptions, timer, and mutexes ---
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex odom_mutex_;
  px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
  rclcpp::Time prev_time_;
  bool odometry_received_ = false;
  bool first_run_ = true;

  // --- Kinematic variables ---
  double phi_raw_=0, theta_raw_=0, psi_raw_=0;
  double p_raw_=0, q_raw_=0, r_raw_=0;
  double p_dot_filt_=0, q_dot_filt_=0, r_dot_filt_=0;
  double phi_dot_raw_calc_=0, theta_dot_raw_calc_=0, psi_dot_raw_calc_=0;

  // --- Filter objects ---
  std::unique_ptr<FilterDiff> fd_p_deriv_, fd_q_deriv_, fd_r_deriv_;

  // --- Dynamixel SDK objects ---
  dynamixel::PortHandler *portHandler_ = nullptr;
  dynamixel::PacketHandler *packetHandler_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;

  // --- Dynamics and Inertia ---
  std::shared_ptr<EqDynamics> dynamics_;
  const Eigen::Matrix3d I_CUAV_, I_CRB_, I_CPR_, I_CYR_;
  Eigen::Matrix3d C_J_LRB_, C_J_LPR_, C_J_LYR_, C_J_I_;

  // --- CSV Logging ---
  std::ofstream log_file_;
  std::string log_file_name_;

  // --- Callbacks ---
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odometry_msg_ = msg;
    odometry_received_ = true;
  }

  // --- Timer Callback ---
  void timer_callback() {
    // --- Timekeeping & Data Acq ---
    rclcpp::Time current_time = this->now();
    double dt = (current_time - prev_time_).seconds();

    // *** CORRECTED dt Check ***
    // Check dt validity - Ensure this condition is on ONE line
    if (dt <= 1e-6 || dt > 0.1) {
        // Optional warning if skipping loops is problematic
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Invalid dt: %.4f, skipping loop.", dt);
        return;
    }
    prev_time_ = current_time;

    px4_msgs::msg::VehicleOdometry::SharedPtr odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!odometry_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No odometry received yet.");
            return;
        }
        odom = latest_odometry_msg_;
    }

    // *** CORRECTED Angular Velocity Extraction ***
    p_raw_ = static_cast<double>(odom->angular_velocity[0]); // Access element [0]
    q_raw_ = static_cast<double>(odom->angular_velocity[1]); // Access element [1]
    r_raw_ = static_cast<double>(odom->angular_velocity[2]); // Access element [2]

    // *** CORRECTED Quaternion Construction ***
    Eigen::Quaterniond q_I_J(
        static_cast<double>(odom->q[0]), // w - Access element [0]
        static_cast<double>(odom->q[1]), // x - Access element [1]
        static_cast<double>(odom->q[2]), // y - Access element [2]
        static_cast<double>(odom->q[3])  // z - Access element [3]
    );
    q_I_J.normalize();
    C_J_I_ = q_I_J.toRotationMatrix().transpose();

    // Extract Euler angles
    extractEulerAnglesZYX(C_J_I_, phi_raw_, theta_raw_, psi_raw_);

    // --- Get Filtered Body Acceleration ---
    if (first_run_) {
        fd_p_deriv_->reset(p_raw_);
        fd_q_deriv_->reset(q_raw_);
        fd_r_deriv_->reset(r_raw_);
        first_run_ = false;
        RCLCPP_INFO(this->get_logger(), "First run: Filters initialized.");
        return;
    }
    fd_p_deriv_->update(p_raw_, dt); fd_q_deriv_->update(q_raw_, dt); fd_r_deriv_->update(r_raw_, dt);
    p_dot_filt_ = fd_p_deriv_->getFilteredDerivative();
    q_dot_filt_ = fd_q_deriv_->getFilteredDerivative();
    r_dot_filt_ = fd_r_deriv_->getFilteredDerivative();
    Eigen::Vector3d omega_JI_raw(p_raw_, q_raw_, r_raw_);
    Eigen::Vector3d omega_JI_dot_filt(p_dot_filt_, q_dot_filt_, r_dot_filt_);

    // --- Calculate Raw Euler Rates ---
    calculateEulerRates(phi_raw_, theta_raw_, p_raw_, q_raw_, r_raw_,
                        phi_dot_raw_calc_, theta_dot_raw_calc_, psi_dot_raw_calc_);

    // --- Update DCMs ---
    update_dcms(phi_raw_, theta_raw_, psi_raw_);

    // --- Calculate Kinematics ---
    Eigen::Vector3d omega_LRB_J_raw(0,0,0);
    Eigen::Vector3d omega_LPR_J_raw(phi_dot_raw_calc_, 0, 0);
    Eigen::Vector3d omega_LPR_LYR_in_LPR_raw(0, theta_dot_raw_calc_, 0);
    Eigen::Vector3d omega_LPR_LYR_in_J_raw = C_J_LPR_ * omega_LPR_LYR_in_LPR_raw;
    Eigen::Vector3d omega_LYR_J_raw = -(omega_LRB_J_raw + omega_LPR_J_raw + omega_LPR_LYR_in_J_raw);

    Eigen::Vector3d omega_dot_LRB_J_filt(0,0,0);
    Eigen::Vector3d omega_dot_LPR_J_filt(0,0,0);
    Eigen::Vector3d omega_dot_LYR_J_filt(0,0,0);

    Eigen::Vector3d omega_UAV_I_inJ_raw = omega_JI_raw;
    Eigen::Vector3d omega_RB_I_inJ_raw  = omega_LRB_J_raw + omega_JI_raw;
    Eigen::Vector3d omega_PR_I_inJ_raw  = omega_LPR_J_raw + omega_JI_raw;
    Eigen::Vector3d omega_YR_I_inJ_raw  = omega_LYR_J_raw + omega_JI_raw;

    Eigen::Vector3d tt_term_PR_raw = omega_JI_raw.cross(omega_LPR_J_raw);
    Eigen::Vector3d tt_term_YR_raw = omega_JI_raw.cross(omega_LYR_J_raw);
    Eigen::Vector3d omega_dot_UAV_I_inJ_filt = omega_JI_dot_filt;
    Eigen::Vector3d omega_dot_RB_I_inJ_filt  = omega_dot_LRB_J_filt + omega_JI_dot_filt;
    Eigen::Vector3d omega_dot_PR_I_inJ_filt  = omega_dot_LPR_J_filt + omega_JI_dot_filt + tt_term_PR_raw;
    Eigen::Vector3d omega_dot_YR_I_inJ_filt  = omega_dot_LYR_J_filt + omega_JI_dot_filt + tt_term_YR_raw;

    // --- Transform to Local Li Frames ---
    Eigen::Matrix3d C_LPR_J=C_J_LPR_.transpose(), C_LYR_J=C_J_LYR_.transpose();
    Eigen::Matrix3d C_LUAV_J=Eigen::Matrix3d::Identity(), C_LRB_J=Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_UAV_I_inLi_raw = C_LUAV_J*omega_UAV_I_inJ_raw;
    Eigen::Vector3d omega_RB_I_inLi_raw  = C_LRB_J*omega_RB_I_inJ_raw;
    Eigen::Vector3d omega_PR_I_inLi_raw  = C_LPR_J*omega_PR_I_inJ_raw;
    Eigen::Vector3d omega_YR_I_inLi_raw  = C_LYR_J*omega_YR_I_inJ_raw;
    Eigen::Vector3d omega_dot_UAV_I_inLi_filt = C_LUAV_J*omega_dot_UAV_I_inJ_filt;
    Eigen::Vector3d omega_dot_RB_I_inLi_filt  = C_LRB_J*omega_dot_RB_I_inJ_filt;
    Eigen::Vector3d omega_dot_PR_I_inLi_filt  = C_LPR_J*omega_dot_PR_I_inJ_filt;
    Eigen::Vector3d omega_dot_YR_I_inLi_filt  = C_LYR_J*omega_dot_YR_I_inJ_filt;

    // --- Calculate Component Torques ---
    Eigen::Vector3d T_UAV = dynamics_->compute_torque_simplified_mixed(I_CUAV_, C_LUAV_J.transpose(), omega_UAV_I_inLi_raw, omega_dot_UAV_I_inLi_filt);
    Eigen::Vector3d T_RB  = dynamics_->compute_torque_simplified_mixed(I_CRB_, C_J_LRB_, omega_RB_I_inLi_raw, omega_dot_RB_I_inLi_filt);
    Eigen::Vector3d T_PR  = dynamics_->compute_torque_simplified_mixed(I_CPR_, C_J_LPR_, omega_PR_I_inLi_raw, omega_dot_PR_I_inLi_filt);
    Eigen::Vector3d T_YR  = dynamics_->compute_torque_simplified_mixed(I_CYR_, C_J_LYR_, omega_YR_I_inLi_raw, omega_dot_YR_I_inLi_filt);

    // --- Total Torque, Jacobian, Current, Send ---
    Eigen::Vector3d total_inertial_torque =  T_RB + T_PR + T_YR;
    Eigen::Vector3d required_motor_torque = total_inertial_torque;
    Eigen::Matrix3d Gamma_inv = compute_inverse_jacobian(phi_raw_, theta_raw_);
    Eigen::Vector3d gimbal_torques = Gamma_inv * required_motor_torque;

    // Apply scaling/zeroing
    double roll_torque=gimbal_torques.x(), pitch_torque=gimbal_torques.y(), yaw_torque=gimbal_torques.z();
    int32_t roll_curr=torque_to_goal_current_calibrated(roll_torque), pitch_curr=torque_to_goal_current_calibrated(pitch_torque), yaw_curr=torque_to_goal_current_calibrated(yaw_torque);

    // --- Log Data to CSV ---
    if (log_file_.is_open() && log_file_.good()) {
        log_file_ << std::fixed << std::setprecision(6) << current_time.seconds() << ","
                  << std::fixed << std::setprecision(4)
                  << p_raw_ << "," << q_raw_ << "," << r_raw_ << ","
                  << p_dot_filt_ << "," << q_dot_filt_ << "," << r_dot_filt_ << ","
                  << total_inertial_torque.x() << "," << total_inertial_torque.y() << "," << total_inertial_torque.z() << ","
                  << gimbal_torques.x() << "," << gimbal_torques.y() << "," << gimbal_torques.z() << ","
                  << roll_torque << "," << pitch_torque << "," << yaw_torque << ","
                  << roll_curr << "," << pitch_curr << "," << yaw_curr
                  << std::endl;
    }

    // --- Send Dynamixel Commands ---
    if (!send_dynamixel_commands(roll_curr, pitch_curr, yaw_curr)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send Dynamixel commands.");
    }

    // --- Optional DEBUG Logging to Console ---
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "CSV Log | Angles(p,t,y): [%.2f,%.2f,%.2f] | Rates(p,q,r): [%.2f,%.2f,%.2f] | Accel(p,q,r): [%.2f,%.2f,%.2f] | TorqCmd: [%.3f,%.3f,%.3f] | CurrCmd: [%d,%d,%d]",
      phi_raw_, theta_raw_, psi_raw_, p_raw_, q_raw_, r_raw_, p_dot_filt_, q_dot_filt_, r_dot_filt_, roll_torque, pitch_torque, yaw_torque, roll_curr, pitch_curr, yaw_curr);
  } // End timer_callback

  // --- Helper Functions ---
  bool initialize_dynamixels() {
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!portHandler_ ||!packetHandler_) {RCLCPP_ERROR(this->get_logger(),"Failed get SDK handlers"); return false;}
    if (!portHandler_->openPort()) { RCLCPP_ERROR(this->get_logger(), "Failed open port: %s", DEVICENAME); return false; }
    if (!portHandler_->setBaudRate(BAUDRATE)) { RCLCPP_ERROR(this->get_logger(), "Failed set baudrate: %d", BAUDRATE); portHandler_->closePort(); return false; }
    RCLCPP_INFO(this->get_logger(), "Opened port %s @ %d baud", DEVICENAME, BAUDRATE);
    for (uint8_t id : DXL_IDS) {
      write_byte_with_log(id, ADDR_TORQUE_ENABLE, 1, "Enable Torque");
       std::this_thread::sleep_for(50ms); // Added small delay after enabling torque
    }
    RCLCPP_INFO(this->get_logger(), "Dynamixels initialized (Torque Enabled). Assumed already in Current Control Mode."); return true;
  }

  void extractEulerAnglesZYX(const Eigen::Matrix3d& R, double& phi, double& theta, double& psi) {
    theta = -asin(R(0, 2));
    theta = std::max(-M_PI_2, std::min(M_PI_2, theta));
    if (abs(cos(theta)) > 1e-6) {
        phi = atan2(R(1, 2), R(2, 2)); psi = atan2(R(0, 1), R(0, 0));
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock detected!");
        phi = 0.0; psi = (theta > 0)? atan2(-R(1,0), -R(1,1)) : atan2(R(1,0), R(1,1));
    }
  }

  void calculateEulerRates(double phi, double theta, double p, double q, double r, double& phi_dot, double& theta_dot, double& psi_dot) {
    double cphi = cos(phi), sphi = sin(phi); double cth = cos(theta);
    if (abs(cth) < 1e-6) {
         RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock! Euler rate calc inaccurate.");
         phi_dot = p; theta_dot = q * cphi - r * sphi; psi_dot = 0;
    } else {
        double tan_th = tan(theta);
        phi_dot = p + q * sphi * tan_th + r * cphi * tan_th;
        theta_dot =     q * cphi          - r * sphi;
        psi_dot =       q * sphi / cth    + r * cphi / cth;
    }
  }

  void update_dcms(double phi, double theta, double /*psi*/) {
    double cphi=cos(phi), sphi=sin(phi), cth=cos(theta), sth=sin(theta);
    C_J_LPR_ << 1,0,0, 0,cphi,sphi, 0,-sphi,cphi;
    C_J_LYR_ << cth,sphi*sth,cphi*sth, 0,cphi,-sphi, -sth,sphi*cth,cphi*cth;
  }

  Eigen::Matrix3d compute_inverse_jacobian(double phi, double theta) {
    double cphi=cos(phi), sphi=sin(phi), cth=cos(theta); Eigen::Matrix3d G_inv=Eigen::Matrix3d::Identity();
    if(abs(cth)<1e-6) {RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),2000,"Gimbal lock! Using Identity InvJac.");}
    else {double tan_th=tan(theta); G_inv << 1,sphi*tan_th,cphi*tan_th, 0,cphi,-sphi, 0,sphi/cth,cphi/cth;}
    return G_inv;
  }

  int32_t torque_to_goal_current_calibrated(double torque_nm) {
    double current_A;
    if (torque_nm >= 0.0) {
      current_A = 0.6370 * torque_nm + 0.0395;
    } else {
      current_A = 0.6439 * torque_nm - 0.1097;
    }
    double raw = (current_A / CURRENT_LSB) + CURRENT_OFFSET_RAW;
    raw = std::clamp(raw, double(CURRENT_RAW_MIN), double(CURRENT_RAW_MAX));
    return static_cast<int32_t>(std::round(raw));
  }

  void write_byte_with_log(uint8_t id, uint16_t addr, uint8_t val, const std::string& op) {
    if (!portHandler_||!packetHandler_) return; uint8_t err=0; int res=packetHandler_->write1ByteTxRx(portHandler_,id,addr,val,&err);
    if(res!=COMM_SUCCESS){RCLCPP_ERROR(this->get_logger(),"ID:%d Addr:%d (%s) Comm Fail: %s",id,addr,op.c_str(),packetHandler_->getTxRxResult(res));}
    else if(err!=0){RCLCPP_ERROR(this->get_logger(),"ID:%d Addr:%d (%s) DXL Error: %s",id,addr,op.c_str(),packetHandler_->getRxPacketError(err));}
  }

  // *** CORRECTED send_dynamixel_commands ***
  bool send_dynamixel_commands(int32_t current_roll, int32_t current_pitch, int32_t current_yaw) {
    if(!groupSyncWrite_ || !packetHandler_) return false;
    bool ok=true;
    uint8_t p[LEN_GOAL_CURRENT]; // Declare buffer array of correct size

    // Roll
    uint16_t w_r=(uint16_t)current_roll;
    p[0]=DXL_LOBYTE(w_r);        // Assign low byte to p[0]
    p[1]=DXL_HIBYTE(w_r);        // Assign high byte to p[1]
    if(!groupSyncWrite_->addParam(DXL_ID_ROLL, p)) ok=false; // Pass array p

    // Pitch
    uint16_t w_p=(uint16_t)current_pitch;
    p[0]=DXL_LOBYTE(w_p);        // Assign low byte to p[0]
    p[1]=DXL_HIBYTE(w_p);        // Assign high byte to p[1]
    if(!groupSyncWrite_->addParam(DXL_ID_PITCH, p)) ok=false; // Pass array p

    // Yaw
    uint16_t w_y=(uint16_t)current_yaw;
    p[0]=DXL_LOBYTE(w_y);        // Assign low byte to p[0]
    p[1]=DXL_HIBYTE(w_y);        // Assign high byte to p[1]
    if(!groupSyncWrite_->addParam(DXL_ID_YAW,  p)) ok=false; // Pass array p

    if(!ok){RCLCPP_ERROR(this->get_logger(),"Failed addParam SyncWrite."); groupSyncWrite_->clearParam(); return false;}
    int res = groupSyncWrite_->txPacket(); bool tx_ok=(res==COMM_SUCCESS);
    if(!tx_ok){RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),1000,"SyncWrite fail: %s", packetHandler_->getTxRxResult(res));}
    groupSyncWrite_->clearParam(); return tx_ok;
  } // End send_dynamixel_commands

}; // End class DroneDynamixelBridgeNode

// --- Main Function ---
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DroneDynamixelBridgeNode> node = nullptr;
  try {
      node = std::make_shared<DroneDynamixelBridgeNode>();
      RCLCPP_INFO(node->get_logger(), "Node created successfully. Spinning...");
      rclcpp::spin(node);
  } catch (const std::runtime_error & e) {
      RCLCPP_FATAL(rclcpp::get_logger("Main"), "Node initialization failed: %s", e.what());
  } catch (const std::exception & e) {
       RCLCPP_FATAL(rclcpp::get_logger("Main"), "An unexpected error occurred: %s", e.what());
  }
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Shutting down node.");
  rclcpp::shutdown();
  return 0;
}
