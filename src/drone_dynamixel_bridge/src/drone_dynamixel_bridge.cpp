#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp> // For body rates
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

// --- Dynamixel Configuration (Write-only) ---
const uint8_t DXL_ID_ROLL = 2, DXL_ID_PITCH = 1, DXL_ID_YAW = 3;
const std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
const char* DEVICENAME = "/dev/ttyUSB1"; // <<< VERIFY this port name
const int BAUDRATE = 4000000;
const float PROTOCOL_VERSION = 2.0;
const uint16_t ADDR_OPERATING_MODE = 11, ADDR_TORQUE_ENABLE = 64, ADDR_GOAL_CURRENT = 102;
const uint16_t LEN_OPERATING_MODE = 1, LEN_TORQUE_ENABLE = 1, LEN_GOAL_CURRENT = 2;
const double CURRENT_STEP = 0.00269; // 2.69 mA per unit (MX-106 2.0)
const double DEFAULT_CURRENT_LIMIT_STEPS = 1193.0; // ~3.21 A (MX-106 2.0 default) - Used for clamping

// --- Filter Differentiator Class ---
// (Only used to get derivative now)
class FilterDiff {
public:
    FilterDiff(double wn, double zeta) : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), initialized_(false) {}
    void update(double input, double dt) {
        if (dt <= 1e-9) { return; }
        if (!initialized_) { x1_ = input; x2_ = 0.0; initialized_ = true; }
        else {
            double x1_dot = x2_;
            double x2_dot = -wn_*wn_*x1_ - 2.0*zeta_*wn_*x2_ + wn_*wn_*input;
            x1_ += x1_dot * dt;
            x2_ += x2_dot * dt;
        }
    }
    double getFilteredDerivative() const { return x2_; }
    void reset(double initial_value = 0.0) { x1_ = initial_value; x2_ = 0.0; initialized_ = true; }
    void resetState(double val, double deriv) { x1_ = val; x2_ = deriv; initialized_ = true; }
private:
    double wn_, zeta_, x1_, x2_; bool initialized_;
};

// --- Dynamics Equations Class ---
class EqDynamics {
public:
    EqDynamics() {}
    Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) const {
        Eigen::Matrix3d m; m << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return m;
    }
    Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &I, const Eigen::Vector3d &omega_dot) const {
        return I * omega_dot;
    }
    Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &I, const Eigen::Vector3d &omega1, const Eigen::Vector3d &omega2) const {
        return omega1.cross(I * omega2);
    }
    // USES RAW VELOCITY AND FILTERED ACCELERATION
    Eigen::Vector3d compute_torque_simplified_mixed(
        const Eigen::Matrix3d &ILi_inLi, const Eigen::Matrix3d &CJLi,
        const Eigen::Vector3d &omega_LiI_inLi_raw, const Eigen::Vector3d &omega_dot_LiI_inLi_filt) const {
        Eigen::Vector3d term1_inLi = computeIDomegaDt(ILi_inLi, omega_dot_LiI_inLi_filt);         // I * alpha_filtered
        Eigen::Vector3d term2_inLi = computeOmegaCrossIomega(ILi_inLi, omega_LiI_inLi_raw, omega_LiI_inLi_raw); // omega_raw x (I * omega_raw)
        return CJLi * (term1_inLi + term2_inLi);
    }
};

// --- ROS2 Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode() : Node("drone_dynamixel_bridge_node"),
    // <<< Inertia Tensors - Ensure in Li frame relative to CoM >>>
    I_CUAV_((Eigen::Matrix3d() <<  0.010679,-0.000001,-0.000072, -0.000001, 0.017318, 0.000007, -0.000072, 0.000007, 0.026610).finished()),
    I_CRB_((Eigen::Matrix3d() <<   0.000847,-0.000004,-0.000010, -0.000004, 0.098249, 0.000000, -0.000010, 0.000000, 0.098096).finished()),
    I_CPR_((Eigen::Matrix3d() <<   0.640302, 0.000043, 0.000078,  0.000043, 0.570491,-0.000025,  0.000078,-0.000025, 1.200050).finished()),
    I_CYR_((Eigen::Matrix3d() <<   2.384163,-0.000049,-0.000104, -0.000049, 1.110062, 0.000781, -0.000104, 0.000781, 1.286313).finished())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Node (Raw Angles/Vel, Axis-Filt Alpha)...");

    // --- Parameters ---
    // <<< Filter ONLY for deriving body acceleration - SEPARATE PARAMS PER AXIS >>>
    double filter_omega_p_deriv = 70.0; // Filter Omega for p_raw -> p_dot_filt <<< TUNE
    double filter_zeta_p_deriv  = 0.7;  // Filter Zeta for p_raw -> p_dot_filt <<< TUNE
    double filter_omega_q_deriv = 70.0; // Filter Omega for q_raw -> q_dot_filt <<< TUNE (Init same as p)
    double filter_zeta_q_deriv  = 0.7;  // Filter Zeta for q_raw -> q_dot_filt <<< TUNE (Init same as p)
    double filter_omega_r_deriv = 70.0; // Filter Omega for r_raw -> r_dot_filt <<< TUNE (Init same as p)
    double filter_zeta_r_deriv  = 0.7;  // Filter Zeta for r_raw -> r_dot_filt <<< TUNE (Init same as p)
    // *** NO OTHER FILTERS NEEDED for state estimation in this version ***

    // --- Subscriptions ---
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1));
    angular_velocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", qos,
        std::bind(&DroneDynamixelBridgeNode::angular_velocity_callback, this, std::placeholders::_1));

    // --- Timer & SDK Init ---
    timer_ = this->create_wall_timer(5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this));
    if (!initialize_dynamixels()) { throw std::runtime_error("Dynamixel initialization failed"); }
    dynamics_ = std::make_shared<EqDynamics>();

    // --- Initialize Filters (ONLY for body accel using separate params) ---
    fd_p_deriv_ = std::make_unique<FilterDiff>(filter_omega_p_deriv, filter_zeta_p_deriv);
    fd_q_deriv_ = std::make_unique<FilterDiff>(filter_omega_q_deriv, filter_zeta_q_deriv);
    fd_r_deriv_ = std::make_unique<FilterDiff>(filter_omega_r_deriv, filter_zeta_r_deriv);

    // --- Sync Write ---
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);
    prev_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Node initialized.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    for (uint8_t id : DXL_IDS) { if (portHandler_ && portHandler_->isOpen() && packetHandler_) {
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, nullptr); rclcpp::sleep_for(10ms); }}
    if (portHandler_ && portHandler_->isOpen()) { portHandler_->closePort(); RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");}
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
    // --- State Variables ---
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex odom_mutex_, ang_vel_mutex_;
    px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
    px4_msgs::msg::VehicleAngularVelocity::SharedPtr latest_angular_velocity_msg_;
    rclcpp::Time prev_time_;
    bool odometry_received_ = false, angular_velocity_received_ = false, first_run_ = true;

    // Kinematics (Raw Angles/Vel, Filtered Accel)
    double phi_raw_=0, theta_raw_=0, psi_raw_=0;             // Raw extracted Euler
    double p_raw_=0, q_raw_=0, r_raw_=0;                      // Raw body rates (omega_JI_raw)
    double p_dot_filt_=0, q_dot_filt_=0, r_dot_filt_=0;      // Filtered body accel (omega_JI_dot_filt)
    double phi_dot_raw_calc_=0, theta_dot_raw_calc_=0, psi_dot_raw_calc_=0; // Euler rates from RAW pqr

    // Filter Objects (Only for deriving body accel)
    std::unique_ptr<FilterDiff> fd_p_deriv_, fd_q_deriv_, fd_r_deriv_;

    // SDK, Dynamics, etc.
    dynamixel::PortHandler *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;
    std::shared_ptr<EqDynamics> dynamics_;
    const Eigen::Matrix3d I_CUAV_, I_CRB_, I_CPR_, I_CYR_;
    Eigen::Matrix3d C_J_LRB_ = Eigen::Matrix3d::Identity(), C_J_LPR_, C_J_LYR_, C_J_I_;

    // --- Callbacks ---
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_); latest_odometry_msg_ = msg; odometry_received_ = true;
    }
    void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ang_vel_mutex_); latest_angular_velocity_msg_ = msg;
        p_raw_ = msg->xyz[0]; q_raw_ = msg->xyz[1]; r_raw_ = msg->xyz[2]; angular_velocity_received_ = true;
    }

    // --- Timer Callback ---
    void timer_callback() {
        // --- Timekeeping & Data Acq ---
        rclcpp::Time current_time = this->now(); double dt = (current_time - prev_time_).seconds();
        if (dt <= 1e-6 || dt > 0.1) { return; } prev_time_ = current_time;
        px4_msgs::msg::VehicleOdometry::SharedPtr odom;
        { std::lock_guard<std::mutex> lock(odom_mutex_); if(!odometry_received_) return; odom = latest_odometry_msg_; }
        { std::lock_guard<std::mutex> lock(ang_vel_mutex_); if(!angular_velocity_received_) return; /* p_raw_ updated */}
        Eigen::Quaterniond q_I_J(odom->q[0], odom->q[1], odom->q[2], odom->q[3]); q_I_J.normalize();
        C_J_I_ = q_I_J.toRotationMatrix().transpose();

        // --- Extract Raw Angles ---
        extractEulerAnglesZYX(C_J_I_, phi_raw_, theta_raw_, psi_raw_); // Raw angles

        // --- Get Filtered Body Acceleration (using separate filters per axis) ---
        if (first_run_) { // Reset filters on first valid loop
            fd_p_deriv_->reset(p_raw_); fd_q_deriv_->reset(q_raw_); fd_r_deriv_->reset(r_raw_);
            first_run_ = false;
            RCLCPP_INFO(this->get_logger(), "First run: Initializing accel filters.");
            return;
        }
        fd_p_deriv_->update(p_raw_, dt); fd_q_deriv_->update(q_raw_, dt); fd_r_deriv_->update(r_raw_, dt);
        p_dot_filt_ = fd_p_deriv_->getFilteredDerivative();
        q_dot_filt_ = fd_q_deriv_->getFilteredDerivative();
        r_dot_filt_ = fd_r_deriv_->getFilteredDerivative();
        Eigen::Vector3d omega_JI_raw(p_raw_, q_raw_, r_raw_);                 // Raw absolute omega
        Eigen::Vector3d omega_JI_dot_filt(p_dot_filt_, q_dot_filt_, r_dot_filt_); // Filtered absolute alpha

        // --- Calculate Raw Euler Rates ---
        calculateEulerRates(phi_raw_, theta_raw_, p_raw_, q_raw_, r_raw_, phi_dot_raw_calc_, theta_dot_raw_calc_, psi_dot_raw_calc_);

        // --- Update DCMs using Raw Angles ---
        update_dcms(phi_raw_, theta_raw_, psi_raw_); // Use raw angles

        // --- Calculate Kinematics (Raw Vel / Filt Accel / Zero Rel Accel) ---
        // Relative Raw Rates (in J) using raw Euler rates
        Eigen::Vector3d omega_LRB_J_raw(0,0,0);
        Eigen::Vector3d omega_LPR_J_raw(phi_dot_raw_calc_, 0, 0); // <<< Use raw calc, VERIFY AXIS X
        Eigen::Vector3d omega_LPR_LYR_in_LPR_raw(0, theta_dot_raw_calc_, 0); // <<< Use raw calc, VERIFY AXIS Y in LPR
        Eigen::Vector3d omega_LPR_LYR_in_J_raw = C_J_LPR_ * omega_LPR_LYR_in_LPR_raw; // Uses DCM from raw angles
        Eigen::Vector3d omega_LYR_J_raw = -(omega_LRB_J_raw + omega_LPR_J_raw + omega_LPR_LYR_in_J_raw);

        // Relative Filtered Accels -> ASSUMED ZERO
        Eigen::Vector3d omega_dot_LRB_J_filt(0,0,0);
        Eigen::Vector3d omega_dot_LPR_J_filt(0,0,0); // Assume zero phi_ddot
        Eigen::Vector3d omega_dot_LYR_J_filt(0,0,0); // Assume zero theta_ddot -> zero rel accel approx
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Assuming ZERO relative angular acceleration.");

        // Absolute Raw Velocity (in J) = omega_LiJ_raw + omega_JI_raw
        Eigen::Vector3d omega_UAV_I_inJ_raw = omega_JI_raw;
        Eigen::Vector3d omega_RB_I_inJ_raw  = omega_LRB_J_raw + omega_JI_raw; // = omega_JI_raw
        Eigen::Vector3d omega_PR_I_inJ_raw  = omega_LPR_J_raw + omega_JI_raw;
        Eigen::Vector3d omega_YR_I_inJ_raw  = omega_LYR_J_raw + omega_JI_raw;

        // Absolute Filtered Acceleration (in J) = omega_dot_LiJ_filt(0) + omega_JI_dot_filt + omega_JI_raw x omega_LiJ_raw
        Eigen::Vector3d tt_term_PR_raw = omega_JI_raw.cross(omega_LPR_J_raw); // <<< Use Raw Omegas
        Eigen::Vector3d tt_term_YR_raw = omega_JI_raw.cross(omega_LYR_J_raw); // <<< Use Raw Omegas
        Eigen::Vector3d omega_dot_UAV_I_inJ_filt = omega_JI_dot_filt;
        Eigen::Vector3d omega_dot_RB_I_inJ_filt  = omega_dot_LRB_J_filt + omega_JI_dot_filt; // = omega_JI_dot_filt
        Eigen::Vector3d omega_dot_PR_I_inJ_filt  = omega_dot_LPR_J_filt + omega_JI_dot_filt + tt_term_PR_raw; // = 0 + omega_JI_dot_filt + tt_term_PR_raw
        Eigen::Vector3d omega_dot_YR_I_inJ_filt  = omega_dot_LYR_J_filt + omega_JI_dot_filt + tt_term_YR_raw; // = 0 + omega_JI_dot_filt + tt_term_YR_raw

        // --- Transform to Local Li Frames ---
        Eigen::Matrix3d C_LPR_J=C_J_LPR_.transpose(), C_LYR_J=C_J_LYR_.transpose(); // Uses DCM from raw angles
        Eigen::Matrix3d C_LUAV_J=Eigen::Matrix3d::Identity(), C_LRB_J=Eigen::Matrix3d::Identity();
        // Raw Velocities in Li
        Eigen::Vector3d omega_UAV_I_inLi_raw = C_LUAV_J*omega_UAV_I_inJ_raw;
        Eigen::Vector3d omega_RB_I_inLi_raw  = C_LRB_J*omega_RB_I_inJ_raw;
        Eigen::Vector3d omega_PR_I_inLi_raw  = C_LPR_J*omega_PR_I_inJ_raw;
        Eigen::Vector3d omega_YR_I_inLi_raw  = C_LYR_J*omega_YR_I_inJ_raw;
        // Filtered Accelerations in Li
        Eigen::Vector3d omega_dot_UAV_I_inLi_filt = C_LUAV_J*omega_dot_UAV_I_inJ_filt;
        Eigen::Vector3d omega_dot_RB_I_inLi_filt  = C_LRB_J*omega_dot_RB_I_inJ_filt;
        Eigen::Vector3d omega_dot_PR_I_inLi_filt  = C_LPR_J*omega_dot_PR_I_inJ_filt;
        Eigen::Vector3d omega_dot_YR_I_inLi_filt  = C_LYR_J*omega_dot_YR_I_inJ_filt;

        // --- Calculate Component Torques (MIXED RAW/FILT) ---
        Eigen::Vector3d T_UAV = dynamics_->compute_torque_simplified_mixed(I_CUAV_, C_LUAV_J.transpose(), omega_UAV_I_inLi_raw, omega_dot_UAV_I_inLi_filt);
        Eigen::Vector3d T_RB  = dynamics_->compute_torque_simplified_mixed(I_CRB_, C_J_LRB_, omega_RB_I_inLi_raw, omega_dot_RB_I_inLi_filt);
        Eigen::Vector3d T_PR  = dynamics_->compute_torque_simplified_mixed(I_CPR_, C_J_LPR_, omega_PR_I_inLi_raw, omega_dot_PR_I_inLi_filt);
        Eigen::Vector3d T_YR  = dynamics_->compute_torque_simplified_mixed(I_CYR_, C_J_LYR_, omega_YR_I_inLi_raw, omega_dot_YR_I_inLi_filt);

        // --- Total Torque, Jacobian, Current, Send ---
        Eigen::Vector3d total_inertial_torque = T_UAV + T_RB + T_PR + T_YR;
        Eigen::Vector3d required_motor_torque = -total_inertial_torque; // Negate to cancel inertia
        Eigen::Matrix3d Gamma_inv = compute_inverse_jacobian(phi_raw_, theta_raw_); // <<< Use raw angles
        Eigen::Vector3d gimbal_torques = Gamma_inv * required_motor_torque;
        double roll_torque=gimbal_torques.x(), pitch_torque=gimbal_torques.y(), yaw_torque=gimbal_torques.z();
        int32_t roll_curr=torque_to_goal_current_calibrated(roll_torque), pitch_curr=torque_to_goal_current_calibrated(pitch_torque), yaw_curr=torque_to_goal_current_calibrated(yaw_torque);
        if (!send_dynamixel_commands(roll_curr, pitch_curr, yaw_curr)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed send Dynamixel commands.");
        }

        // --- Logging ---
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, // Log @ 10 Hz
          "AnglesRaw(p,t,y): [%.2f,%.2f,%.2f] | RatesRaw(p,q,r): [%.2f,%.2f,%.2f] | AccelFilt(p,q,r): [%.2f,%.2f,%.2f] | E RatesRaw(p,t): [%.2f,%.2f] | TorqCmd: [%.3f,%.3f,%.3f] | CurrCmd: [%ld,%ld,%ld]",
          phi_raw_, theta_raw_, psi_raw_,         // Log raw angles
          p_raw_, q_raw_, r_raw_,                 // Log raw body rates
          p_dot_filt_, q_dot_filt_, r_dot_filt_, // Log filtered body accels
          phi_dot_raw_calc_, theta_dot_raw_calc_, // Log euler rates from raw pqr
          roll_torque, pitch_torque, yaw_torque, roll_curr, pitch_curr, yaw_curr);
    }

    // --- Helper Functions ---
    bool initialize_dynamixels() {
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        if (!portHandler_ || !packetHandler_) {RCLCPP_ERROR(this->get_logger(),"Failed get SDK handlers"); return false;}
        if (!portHandler_->openPort()) { RCLCPP_ERROR(this->get_logger(), "Failed open port: %s", DEVICENAME); return false; }
        if (!portHandler_->setBaudRate(BAUDRATE)) { RCLCPP_ERROR(this->get_logger(), "Failed set baudrate: %d", BAUDRATE); portHandler_->closePort(); return false; }
        RCLCPP_INFO(this->get_logger(), "Opened port %s @ %d baud", DEVICENAME, BAUDRATE);
        uint8_t op_mode = 0; // Current Control
        for (uint8_t id : DXL_IDS) { write_byte_with_log(id, ADDR_OPERATING_MODE, op_mode, "Set OpMode"); rclcpp::sleep_for(30ms); }
        for (uint8_t id : DXL_IDS) { write_byte_with_log(id, ADDR_TORQUE_ENABLE, 1, "Enable Torque"); }
        RCLCPP_INFO(this->get_logger(), "Dynamixels initialized (OpMode=CurrentControl, Torque=Enabled)."); return true;
    }

    // Assumes ZYX rotation sequence for C_J_I (Body from Inertial)
    void extractEulerAnglesZYX(const Eigen::Matrix3d& R_Body_I, double& phi, double& theta, double& psi) {
        // R_Body_I(2,0) = sin(theta)
        // R_Body_I(1,0) = -cos(theta)sin(phi)
        // R_Body_I(0,0) = cos(theta)cos(phi)
        // R_Body_I(2,1) = cos(theta)sin(psi)
        // R_Body_I(2,2) = cos(theta)cos(psi)

        // Note: Standard ZYX convention often uses Ry(theta) then Rx(phi).
        // Let's assume the convention matches C_J_I_ = q_I_J.toRotationMatrix().transpose()
        // which gives rotation FROM Inertial TO Body.
        // If R is C_J_I (Body from Inertial):
        theta = asin(R_Body_I(0, 2)); // More standard ZYX: asin(-R(2,0))? Check PX4 frame vs Standard frame
                                      // Let's try the direct interpretation of PDF Eq 18c first: C_J_I
                                      // theta = asin(-R(0,2)) from PDF Eq 18c C_J_I(0,2) = -sth
        theta = -asin(R_Body_I(0, 2)); // Using element (0,2) = -sin(theta)

        if (abs(cos(theta)) > 1e-6) { // Avoid gimbal lock singularity
            // phi = atan2(R_Body_I(1,2), R_Body_I(2,2)); // From sin(phi)cos(theta), cos(phi)cos(theta)
            phi = atan2(R_Body_I(1, 2), R_Body_I(2, 2)); // Using elements (1,2) and (2,2)
            // psi = atan2(R_Body_I(0,1), R_Body_I(0,0)); // From cos(theta)sin(psi), cos(theta)cos(psi)
            psi = atan2(R_Body_I(0, 1), R_Body_I(0, 0)); // Using elements (0,1) and (0,0)

        } else { // Gimbal lock case
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock detected! Euler angles inaccurate.");
            phi = 0.0; // Set roll arbitrarily to 0
            // psi = atan2(-R_Body_I(1,0), R_Body_I(1,1)); // Or similar based on actual lock condition elements
            psi = (theta > 0) ? atan2(-R_Body_I(1,0), R_Body_I(1,1)) : atan2(R_Body_I(1,0), -R_Body_I(1,1)); // Adjusted for asin sign convention
        }
    }


    // Calculates Euler rates from body rates (p,q,r) and angles (phi, theta)
    void calculateEulerRates(double phi, double theta, double p, double q, double r,
                             double& phi_dot, double& theta_dot, double& psi_dot) {
        double cphi = cos(phi), sphi = sin(phi);
        double cth = cos(theta);

        if (abs(cth) < 1e-6) { // Handle gimbal lock singularity
             RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock! Euler rate calculation inaccurate/approximate.");
             phi_dot = p; // Approximation
             theta_dot = q * cphi - r * sphi; // Approximation
             psi_dot = 0; // Undefined
        } else {
            double tan_th = tan(theta); // Safe because abs(cth) > 1e-6
            phi_dot = p + q * sphi * tan_th + r * cphi * tan_th;
            theta_dot =     q * cphi          - r * sphi;
            psi_dot =       q * sphi / cth    + r * cphi / cth;
        }
    }

    // Updates DCMs based on RAW gimbal angles
    void update_dcms(double phi, double theta, double /*psi*/) { // psi not needed for C_J_LPR/YR
        double cphi=cos(phi), sphi=sin(phi), cth=cos(theta), sth=sin(theta);
        // C_J/LPR = RotX(phi) <<< VERIFY AXIS
        C_J_LPR_ << 1,0,0, 0,cphi,sphi, 0,-sphi,cphi;
        // C_J/LYR from PDF Eq 18b
        C_J_LYR_ << cth,sphi*sth,cphi*sth, 0,cphi,-sphi, -sth,sphi*cth,cphi*cth;
    }

    // Computes inverse Jacobian using RAW gimbal angles
    Eigen::Matrix3d compute_inverse_jacobian(double phi, double theta) {
        double cphi=cos(phi), sphi=sin(phi), cth=cos(theta); Eigen::Matrix3d G_inv=Eigen::Matrix3d::Identity();
        if(abs(cth)<1e-6) {RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),2000,"Gimbal lock! Using Identity InvJac.");}
        else {double tan_th=tan(theta); G_inv << 1,sphi*tan_th,cphi*tan_th, 0,cphi,-sphi, 0,sphi/cth,cphi/cth;}
        return G_inv;
    }

    // Converts torque [Nm] to Dynamixel goal current steps using CALIBRATED fits
    int32_t torque_to_goal_current_calibrated(double torque_nm) {
        double current_A;
        if (torque_nm >= 0) { current_A = 0.6370 * torque_nm + 0.0395; } // Positive Torque
        else { current_A = 0.6439 * torque_nm - 0.1097; } // Negative Torque
        double steps = current_A / CURRENT_STEP;
        double max_steps = DEFAULT_CURRENT_LIMIT_STEPS;
        steps = std::max(-max_steps, std::min(max_steps, steps)); // Clamp
        return static_cast<int32_t>(std::round(steps));
    }

    // Helper to write single byte with logging
    void write_byte_with_log(uint8_t id, uint16_t addr, uint8_t val, const std::string& op) {
        if (!portHandler_||!packetHandler_) return; uint8_t err=0; int res=packetHandler_->write1ByteTxRx(portHandler_,id,addr,val,&err);
        if(res!=COMM_SUCCESS){RCLCPP_ERROR(this->get_logger(),"ID:%d Addr:%d (%s) Comm Fail: %s",id,addr,op.c_str(),packetHandler_->getTxRxResult(res));}
        else if(err!=0){RCLCPP_ERROR(this->get_logger(),"ID:%d Addr:%d (%s) DXL Error: %s",id,addr,op.c_str(),packetHandler_->getRxPacketError(err));}
        // else { RCLCPP_DEBUG(this->get_logger(), "%s (ID %d) -> 0x%02X success", op.c_str(), id, (unsigned)val); } // DEBUG level log
    }

    // Sends current commands via SyncWrite
    bool send_dynamixel_commands(int32_t current_roll, int32_t current_pitch, int32_t current_yaw) {
        if(!groupSyncWrite_||!packetHandler_) return false; bool ok=true; uint8_t p[LEN_GOAL_CURRENT];
        // Roll
        uint16_t w_r=(uint16_t)current_roll; p[0]=DXL_LOBYTE(w_r);p[1]=DXL_HIBYTE(w_r); if(!groupSyncWrite_->addParam(DXL_ID_ROLL, p)) ok=false;
        // Pitch
        uint16_t w_p=(uint16_t)current_pitch; p[0]=DXL_LOBYTE(w_p);p[1]=DXL_HIBYTE(w_p); if(!groupSyncWrite_->addParam(DXL_ID_PITCH,p)) ok=false;
        // Yaw
        uint16_t w_y=(uint16_t)current_yaw; p[0]=DXL_LOBYTE(w_y);p[1]=DXL_HIBYTE(w_y); if(!groupSyncWrite_->addParam(DXL_ID_YAW,  p)) ok=false;
        if(!ok){RCLCPP_ERROR(this->get_logger(),"Failed addParam SyncWrite."); groupSyncWrite_->clearParam(); return false;}
        int res = groupSyncWrite_->txPacket(); bool tx_ok=(res==COMM_SUCCESS);
        if(!tx_ok){RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),1000,"SyncWrite fail: %s", packetHandler_->getTxRxResult(res));}
        groupSyncWrite_->clearParam(); return tx_ok;
    }

}; // End class DroneDynamixelBridgeNode

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
