#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h> // Includes group_sync_write.h and group_bulk_read.h
#include <chrono>
#include <mutex>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept> // Required for std::runtime_error
#include <algorithm> // Required for std::max, std::min
#include <Eigen/Dense> // Ensure Eigen is linked (find_package(Eigen3 REQUIRED))

// For timer literals and chrono durations
using namespace std::chrono_literals;

// --- Dynamixel Configuration ---
// IDs
const uint8_t DXL_ID_ROLL  = 2;
const uint8_t DXL_ID_PITCH = 1;
const uint8_t DXL_ID_YAW   = 3;
const std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
// Port & Baud
const char* DEVICENAME     = "/dev/ttyUSB1"; // <<< VERIFY this port name
const int   BAUDRATE       = 4000000;
const float PROTOCOL_VERSION = 2.0;
// Addresses (Verified for MX-106 2.0 from E-Manual)
const uint16_t ADDR_OPERATING_MODE   = 11;
const uint16_t ADDR_CURRENT_LIMIT    = 38;
const uint16_t ADDR_TORQUE_ENABLE    = 64;
const uint16_t ADDR_GOAL_CURRENT     = 102;
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t ADDR_PRESENT_CURRENT  = 126;
// Data Lengths (Verified for MX-106 2.0)
const uint16_t LEN_OPERATING_MODE   = 1;
const uint16_t LEN_TORQUE_ENABLE    = 1;
const uint16_t LEN_GOAL_CURRENT     = 2; // Signed value, transmitted as 2 bytes
const uint16_t LEN_PRESENT_VELOCITY = 4; // Signed value, 4 bytes
const uint16_t LEN_PRESENT_POSITION = 4; // Signed value, 4 bytes (Note: Manual says 4 bytes for Pos)
const uint16_t LEN_PRESENT_CURRENT  = 2; // Signed value, 2 bytes
const uint16_t LEN_CURRENT_LIMIT    = 2; // Unsigned value, 2 bytes
// Conversion Factors (Values for MX-106 2.0 from E-Manual)
const double VELOCITY_TO_RAD_PER_SEC = 0.229 * (2.0 * M_PI / 60.0); // 0.229 RPM per unit -> rad/s <<< VERIFY 0.229 UNIT
const double POSITION_TO_RAD         = (2.0 * M_PI / 4096.0);     // 4096 units per revolution -> rad <<< VERIFY 4096 STEPS/REV & ZERO OFFSET
const double CURRENT_STEP            = 0.00269;                   // 2.69 mA per unit for Goal/Present Current (MX-106 2.0)
// Torque Constant (Estimated from Stall Torque/Current in E-Manual)
const double TORQUE_CONSTANT_APPROX  = 8.4 / 5.2;                 // ~1.615 Nm/A (Stall conditions) <<< CALIBRATION RECOMMENDED
// Default Current Limit value (used for clamping)
const double DEFAULT_CURRENT_LIMIT_STEPS = 1193.0;                // Corresponds to ~3.21 A (MX-106 2.0 default)


// --- Filter Differentiator Class ---
class FilterDiff {
public:
    FilterDiff(double wn, double zeta) : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), initialized_(false) {}
    void update(double input, double dt) {
        if (dt <= 1e-9) { // Prevent division by zero or instability with tiny/negative dt
             // Optionally log a warning here if dt is consistently invalid
            return;
        }
        if (!initialized_) {
            x1_ = input; x2_ = 0.0; initialized_ = true;
        } else {
            // Euler integration
            double x1_dot = x2_;
            double x2_dot = -wn_ * wn_ * x1_ - 2.0 * zeta_ * wn_ * x2_ + wn_ * wn_ * input;
            x1_ += x1_dot * dt;
            x2_ += x2_dot * dt;
        }
    }
    double getFilteredValue() const { return x1_; }
    double getFilteredDerivative() const { return x2_; }
    void reset(double initial_value = 0.0) { x1_ = initial_value; x2_ = 0.0; initialized_ = true; }
private:
    double wn_, zeta_, x1_, x2_; bool initialized_;
};

// --- Theoretical Dynamics Equations Class ---
class EqDynamics {
public:
    EqDynamics() {}

    // Create skew-symmetric matrix for cross product v x (...) = crossProductMatrix(v) * (...)
    Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) const { // Mark as const
        Eigen::Matrix3d m;
        m <<       0.0, -v.z(),  v.y(),
              v.z(),      0.0, -v.x(),
             -v.y(),   v.x(),      0.0;
        return m;
    }

    // Compute IJ = C_Ji * I_Ci_in_Li * C_Ji^T (Transforms inertia from Li frame to J frame)
    Eigen::Matrix3d computeInertiaInJ(const Eigen::Matrix3d &I_Ci_in_Li, const Eigen::Matrix3d &C_Ji) const { // Mark as const
        return C_Ji * I_Ci_in_Li * C_Ji.transpose();
    }

    // Compute I * omega_dot (all in frame J)
    Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &I_in_J, const Eigen::Vector3d &omega_dot_in_J) const { // Mark as const
        return I_in_J * omega_dot_in_J;
    }

    // Compute omega1 x (I * omega2) - all vectors and tensor in frame J
    Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &I_in_J,
                                            const Eigen::Vector3d &omega1_in_J,
                                            const Eigen::Vector3d &omega2_in_J) const { // Mark as const
        return omega1_in_J.cross(I_in_J * omega2_in_J);
    }

    // Calculates INERTIAL torque generated BY ONE COMPONENT's motion
    // <<< NEEDS RIGOROUS VERIFICATION AGAINST PDF DERIVATION (Eq 16 applied component-wise) >>>
    Eigen::Vector3d computeComponentInertialTorque(
        const Eigen::Matrix3d &I_Ci_in_Li, // Inertia of component i relative to its CoM, expressed in frame Li
        const Eigen::Matrix3d &C_Ji,       // Rotation matrix from component frame Li to base frame J
        const Eigen::Vector3d &omega_JI,   // Absolute angular velocity of J wrt I, expressed in J
        const Eigen::Vector3d &omega_LiJ,  // Relative angular velocity of Li wrt J, expressed in J
        const Eigen::Vector3d &omega_JI_dot, // Absolute angular acceleration d(omega_JI)/dt, expressed in J
        const Eigen::Vector3d &omega_LiJ_dot // Relative angular acceleration d(omega_LiJ)/dt, expressed in J (NEEDS VERIFICATION of calculation method)
    ) const // Mark as const
    {
        Eigen::Matrix3d IJ_i = computeInertiaInJ(I_Ci_in_Li, C_Ji); // Component inertia in J frame

        // Summing terms based on structure of PDF Eq 16, applied to component 'i'
        Eigen::Vector3d term_abs_accel = computeIDomegaDt(IJ_i, omega_JI_dot);
        Eigen::Vector3d term_abs_gyro  = computeOmegaCrossIomega(IJ_i, omega_JI, omega_JI);
        Eigen::Vector3d term_rel_accel = computeIDomegaDt(IJ_i, omega_LiJ_dot);
        Eigen::Vector3d term_interact1 = computeOmegaCrossIomega(IJ_i, omega_JI, omega_LiJ);
        Eigen::Vector3d term_interact2 = computeOmegaCrossIomega(IJ_i, omega_LiJ, omega_JI);
        Eigen::Vector3d term_rel_gyro  = computeOmegaCrossIomega(IJ_i, omega_LiJ, omega_LiJ);

        // Note: This summation structure matches terms in Eq 16 but lacks potential dI/dt terms. Verify against PDF.
        Eigen::Vector3d component_torque = term_abs_accel + term_abs_gyro
                                         + term_rel_accel + term_rel_gyro
                                         + term_interact1 + term_interact2;

         return component_torque;
    }
};


// --- ROS2 Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode()
  : Node("drone_dynamixel_bridge_node"),
    // <<< MUST INITIALIZE WITH CORRECT INERTIA TENSORS FROM PDF TABLE 2 >>>
    // <<< AND VERIFY/TRANSFORM TO BE EXPRESSED IN LOCAL (Li) FRAME >>>
    I_CUAV_((Eigen::Matrix3d() <<  0.010679, -0.000001, -0.000072, // Example values from PDF
                                 -0.000001,  0.017318,  0.000007,
                                 -0.000072,  0.000007,  0.026610).finished()),
    I_CRB_((Eigen::Matrix3d() <<   0.000847, -0.000004, -0.000010, // Example values from PDF
                                 -0.000004,  0.098249,  0.000000,
                                 -0.000010,  0.000000,  0.098096).finished()),
    I_CPR_((Eigen::Matrix3d() <<   0.640302,  0.000043,  0.000078, // Example values from PDF
                                  0.000043,  0.570491, -0.000025,
                                  0.000078, -0.000025,  1.200050).finished()),
    I_CYR_((Eigen::Matrix3d() <<   2.384163, -0.000049, -0.000104, // Example values from PDF
                                 -0.000049,  1.110062,  0.000781,
                                 -0.000104,  0.000781,  1.286313).finished()),
    // Initialize DCMs to identity initially
    C_J_LPR_(Eigen::Matrix3d::Identity()),
    C_J_LYR_(Eigen::Matrix3d::Identity()),
    C_J_I_(Eigen::Matrix3d::Identity())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Drone Dynamixel Bridge Node...");
    RCLCPP_INFO(this->get_logger(), "Using Inertia Tensor values from PDF Table 2 - VERIFY FRAME!");

    // --- Parameters ---
    // <<< TUNE THESE filters using logged data (Absolute & Relative separately) >>>
    double filter_omega = 70.0;
    double filter_zeta = 0.7;

    // --- Subscriptions ---
    auto qos = rclcpp::QoS(10).best_effort();
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      std::bind(&DroneDynamixelBridgeNode::odometry_callback, this, std::placeholders::_1)
    );

    // --- Timer ---
    timer_ = this->create_wall_timer(5ms, std::bind(&DroneDynamixelBridgeNode::timer_callback, this)); // 200 Hz

    // --- Dynamixel SDK Initialization ---
    if (!initialize_dynamixels()) {
        throw std::runtime_error("Dynamixel initialization failed");
    }

    // --- Initialize Dynamics Model ---
    dynamics_ = std::make_shared<EqDynamics>();

    // --- Initialize Filters ---
    fd_roll_abs_  = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_pitch_abs_ = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_yaw_abs_   = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_roll_rel_  = std::make_unique<FilterDiff>(filter_omega, filter_zeta); // <<< NEEDS TUNING
    fd_pitch_rel_ = std::make_unique<FilterDiff>(filter_omega, filter_zeta); // <<< NEEDS TUNING
    fd_yaw_rel_   = std::make_unique<FilterDiff>(filter_omega, filter_zeta); // <<< NEEDS TUNING

    // --- Initialize Bulk Read/Sync Write ---
    groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
    if (!setup_bulk_read()) { throw std::runtime_error("Failed to setup Dynamixel BulkRead"); }
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    prev_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "DroneDynamixelBridgeNode initialized successfully.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    // Ensure torque is disabled before closing port
    for (uint8_t id : DXL_IDS) {
        // Ignore potential errors during shutdown sequence
        if (portHandler_ && portHandler_->is_open() && packetHandler_) {
             packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, nullptr);
             rclcpp::sleep_for(10ms); // Short delay
        }
    }
    if (portHandler_ && portHandler_->is_open()) {
        portHandler_->closePort();
         RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");
    }
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
    // --- State Variables ---
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_; // Protects odometry message access
    px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
    rclcpp::Time prev_time_;
    bool odometry_received_ = false;

    // Raw Dynamixel Readings
    int32_t raw_velocity_roll_ = 0, raw_velocity_pitch_ = 0, raw_velocity_yaw_ = 0;
    int32_t raw_position_roll_ = 0, raw_position_pitch_ = 0, raw_position_yaw_ = 0;

    // Processed Angles (Relative - rad)
    double current_phi_ = 0.0, current_theta_ = 0.0, current_psi_ = 0.0;

    // Filter Objects
    std::unique_ptr<FilterDiff> fd_roll_abs_, fd_pitch_abs_, fd_yaw_abs_;
    std::unique_ptr<FilterDiff> fd_roll_rel_, fd_pitch_rel_, fd_yaw_rel_;

    // Dynamixel SDK Objects
    dynamixel::PortHandler *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;

    // Dynamics Model & Properties
    std::shared_ptr<EqDynamics> dynamics_;
    // Inertia Tensors (relative to CoM, expressed in local frame Li - ASSUMPTION)
    const Eigen::Matrix3d I_CUAV_;
    const Eigen::Matrix3d I_CRB_;
    const Eigen::Matrix3d I_CPR_;
    const Eigen::Matrix3d I_CYR_;
    // Rotation Matrices (Calculated dynamically)
    Eigen::Matrix3d C_J_LRB_ = Eigen::Matrix3d::Identity(); // J = LRB frame
    Eigen::Matrix3d C_J_LPR_;
    Eigen::Matrix3d C_J_LYR_;
    Eigen::Matrix3d C_J_I_;


    // --- Callbacks ---
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odometry_msg_ = msg;
        odometry_received_ = true;
    }

    // --- Timer Callback (Main Loop) ---
    void timer_callback() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 1e-6 || dt > 0.1) { // Check for valid timestep
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestep dt = %.4f s. Skipping update.", dt);
             prev_time_ = current_time; return;
        }
        prev_time_ = current_time;

        // --- 1. Read Dynamixel Data ---
        if (!read_dynamixel_data()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dynamixel read failed. Skipping update.");
            return; // Skip cycle if read fails
        }

        // --- 2. Process Dynamixel Readings ---
        double phi_dot_raw   = static_cast<double>(raw_velocity_roll_) * VELOCITY_TO_RAD_PER_SEC;
        double theta_dot_raw = static_cast<double>(raw_velocity_pitch_) * VELOCITY_TO_RAD_PER_SEC;
        double psi_dot_raw   = static_cast<double>(raw_velocity_yaw_) * VELOCITY_TO_RAD_PER_SEC;
        // <<< ADJUST ZERO OFFSET if necessary by subtracting the zero reading >>>
        double zero_offset_phi = 0.0; // Example
        double zero_offset_theta = 0.0; // Example
        double zero_offset_psi = 0.0; // Example
        current_phi_   = (static_cast<double>(raw_position_roll_) * POSITION_TO_RAD) - zero_offset_phi;
        current_theta_ = (static_cast<double>(raw_position_pitch_) * POSITION_TO_RAD) - zero_offset_theta;
        current_psi_   = (static_cast<double>(raw_position_yaw_) * POSITION_TO_RAD) - zero_offset_psi;


        // --- 3. Get Absolute Velocity from Odometry ---
        px4_msgs::msg::VehicleOdometry::SharedPtr odom;
        { std::lock_guard<std::mutex> lock(mutex_);
          if (!odometry_received_) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Odometry...");
              return;
          }
          odom = latest_odometry_msg_;
        }
        Eigen::Vector3d omega_JI_raw(odom->angular_velocity[0], odom->angular_velocity[1], odom->angular_velocity[2]);

        // --- 4. Filter Velocities ---
        fd_roll_abs_->update(omega_JI_raw.x(), dt); fd_pitch_abs_->update(omega_JI_raw.y(), dt); fd_yaw_abs_->update(omega_JI_raw.z(), dt);
        Eigen::Vector3d omega_JI(fd_roll_abs_->getFilteredValue(), fd_pitch_abs_->getFilteredValue(), fd_yaw_abs_->getFilteredValue());
        Eigen::Vector3d omega_JI_dot(fd_roll_abs_->getFilteredDerivative(), fd_pitch_abs_->getFilteredDerivative(), fd_yaw_abs_->getFilteredDerivative());

        fd_roll_rel_->update(phi_dot_raw, dt); fd_pitch_rel_->update(theta_dot_raw, dt); fd_yaw_rel_->update(psi_dot_raw, dt);
        double phi_dot_filt   = fd_roll_rel_->getFilteredValue(); double theta_dot_filt = fd_pitch_rel_->getFilteredValue(); double psi_dot_filt   = fd_yaw_rel_->getFilteredValue();
        double phi_ddot_filt  = fd_roll_rel_->getFilteredDerivative(); double theta_ddot_filt= fd_pitch_rel_->getFilteredDerivative(); double psi_ddot_filt  = fd_yaw_rel_->getFilteredDerivative();

        // --- 5. Calculate Relative Vectors in Frame J ---
        // <<< VERIFY AXIS CONVENTIONS for mapping phi_dot etc. to vector components >>>
        Eigen::Vector3d omega_LRB_J(0,0,0);
        Eigen::Vector3d omega_LPR_J(phi_dot_filt, 0, 0); // Assuming roll motor rotates about X axis of J frame
        Eigen::Vector3d omega_LYR_J = compute_omega_LYR_J(phi_dot_filt, theta_dot_filt); // Uses DCMs internally

        Eigen::Vector3d omega_dot_LRB_J(0,0,0);
        Eigen::Vector3d omega_dot_LPR_J(phi_ddot_filt, 0, 0); // Assuming roll motor rotates about X axis of J frame
        Eigen::Vector3d omega_dot_LYR_J = compute_omega_dot_LYR_J(phi_dot_filt, theta_dot_filt, phi_ddot_filt, theta_ddot_filt); // <<< NEEDS DERIVATION VERIFICATION

        // --- 6. Calculate DCMs ---
        update_dcms(current_phi_, current_theta_, current_psi_);

        // --- 7. Calculate Component Inertial Torques ---
        // <<< THIS SECTION REQUIRES RIGOROUS VALIDATION AGAINST PDF DERIVATIONS >>>
        // Assuming I_C*_ are correctly defined in Li frame.
        // Note: UAV has no relative motion wrt J (omega_LiJ = 0, omega_LiJ_dot = 0)
        Eigen::Vector3d T_UAV_inertia = dynamics_->computeComponentInertialTorque(I_CUAV_, Eigen::Matrix3d::Identity(), omega_JI, Eigen::Vector3d::Zero(), omega_JI_dot, Eigen::Vector3d::Zero());
        // Note: Roll bar has no relative motion wrt J (omega_LiJ = 0, omega_LiJ_dot = 0)
        Eigen::Vector3d T_RB = dynamics_->computeComponentInertialTorque(I_CRB_, C_J_LRB_, omega_JI, omega_LRB_J, omega_JI_dot, omega_dot_LRB_J);
        Eigen::Vector3d T_PR = dynamics_->computeComponentInertialTorque(I_CPR_, C_J_LPR_, omega_JI, omega_LPR_J, omega_JI_dot, omega_dot_LPR_J);
        Eigen::Vector3d T_YR = dynamics_->computeComponentInertialTorque(I_CYR_, C_J_LYR_, omega_JI, omega_LYR_J, omega_JI_dot, omega_dot_LYR_J);

        // --- 8. Calculate Total Cancellation Torque ---
        Eigen::Vector3d total_inertial_torque = T_UAV_inertia + T_RB + T_PR + T_YR;
        Eigen::Vector3d required_motor_torque = -total_inertial_torque; // <<< VERIFY SIGN CONVENTION

        // --- 9. Convert Torque Vector to Individual Motor Torques (Mx, My, Mz) ---
        Eigen::Matrix3d Gamma = compute_inverse_jacobian(current_phi_, current_theta_);
        Eigen::Vector3d gimbal_torques = Gamma * required_motor_torque;

        double roll_motor_torque_cmd  = gimbal_torques.x(); // Torque around Roll axis
        double pitch_motor_torque_cmd = gimbal_torques.y(); // Torque around Pitch axis
        double yaw_motor_torque_cmd   = gimbal_torques.z(); // Torque around Yaw axis

        // --- 10. Convert Nm to Goal Current ---
        int32_t goal_current_roll  = torque_to_goal_current(roll_motor_torque_cmd);
        int32_t goal_current_pitch = torque_to_goal_current(pitch_motor_torque_cmd);
        int32_t goal_current_yaw   = torque_to_goal_current(yaw_motor_torque_cmd);

        // --- 11. Send Commands ---
        if (!send_dynamixel_commands(goal_current_roll, goal_current_pitch, goal_current_yaw)) {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send Dynamixel commands.");
        }

        // --- Logging ---
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, // Use DEBUG level
          "Angles(p,t,y): [%.2f,%.2f,%.2f] | VelJI: [%.2f,%.2f,%.2f] | VelLiJ(p,t,y): [%.2f,%.2f,%.2f] | TorqueCmd: [%.3f,%.3f,%.3f] | CurrCmd: [%ld,%ld,%ld]",
          current_phi_, current_theta_, current_psi_,
          omega_JI.x(), omega_JI.y(), omega_JI.z(),
          phi_dot_filt, theta_dot_filt, psi_dot_filt, // Using filtered rel rates for log
          roll_motor_torque_cmd, pitch_motor_torque_cmd, yaw_motor_torque_cmd,
          goal_current_roll, goal_current_pitch, goal_current_yaw);

    } // End timer_callback

    // --- Helper Functions ---

    bool initialize_dynamixels() {
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        if (!portHandler_) { RCLCPP_ERROR(this->get_logger(), "Failed to get Port Handler"); return false; }
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        if (!packetHandler_) { RCLCPP_ERROR(this->get_logger(), "Failed to get Packet Handler"); return false; }

        if (!portHandler_->openPort()) { RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICENAME); return false; }
        RCLCPP_INFO(this->get_logger(), "Opened Dynamixel port: %s", DEVICENAME);
        if (!portHandler_->setBaudRate(BAUDRATE)) { RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE); portHandler_->closePort(); return false; }
        RCLCPP_INFO(this->get_logger(), "Set baudrate: %d", BAUDRATE);

        // Set Operating Mode to Current Control (0) for all motors
        uint8_t current_control_mode = 0;
        for (uint8_t id : DXL_IDS) {
            RCLCPP_INFO(this->get_logger(), "Setting OpMode=CurrentControl for ID %d...", id);
            write_byte_with_log(id, ADDR_OPERATING_MODE, current_control_mode, "Set OpMode");
            rclcpp::sleep_for(30ms); // Allow time for mode change and EEPROM write if applicable
        }

        // Enable Torque for all motors
        for (uint8_t id : DXL_IDS) {
             RCLCPP_INFO(this->get_logger(), "Enabling Torque for ID %d...", id);
            write_byte_with_log(id, ADDR_TORQUE_ENABLE, 1, "Enable Torque");
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixels initialized (OpMode=CurrentControl, Torque=Enabled).");
        return true;
    }

    bool setup_bulk_read() {
        bool overall_result = true;
        // Read Vel (4 bytes) + Pos (4 bytes) starting from Present Velocity addr
        uint16_t read_start_addr = ADDR_PRESENT_VELOCITY;
        uint16_t read_len = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION; // Total 8 bytes
        for(uint8_t id : DXL_IDS) {
            if (!groupBulkRead_->addParam(id, read_start_addr, read_len)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add BulkRead param for ID %d", id);
                overall_result = false;
            }
        }
        if (!overall_result) {
            RCLCPP_ERROR(this->get_logger(), "One or more BulkRead parameter additions failed!");
        }
        return overall_result;
    }

    bool read_dynamixel_data() {
        int dxl_comm_result = groupBulkRead_->txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "BulkRead txRxPacket failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
            // Consider clearing params? groupBulkRead_->clearParam(); But might need setup again.
            return false;
        }

        bool got_all_data = true;
        uint16_t addr_vel = ADDR_PRESENT_VELOCITY;
        uint16_t len_vel = LEN_PRESENT_VELOCITY;
        uint16_t addr_pos = ADDR_PRESENT_POSITION;
        uint16_t len_pos = LEN_PRESENT_POSITION;

        for (uint8_t id : DXL_IDS) {
            // Check if data for this ID is available (essential!)
            if (!groupBulkRead_->isAvailable(id, addr_vel, len_vel + len_pos)) {
                RCLCPP_ERROR(this->get_logger(), "BulkRead data not available for ID %d (Addr %d, Len %d)", id, addr_vel, len_vel + len_pos);
                got_all_data = false;
                continue; // Try to get data for other IDs
            }
            // Read velocity first
            // Use getData(ID, Address, Length) -> returns uint32_t, needs casting
            int32_t vel_raw = static_cast<int32_t>(groupBulkRead_->getData(id, addr_vel, len_vel));
            // Read position next
            int32_t pos_raw = static_cast<int32_t>(groupBulkRead_->getData(id, addr_pos, len_pos));

            // Assign based on ID
            if (id == DXL_ID_ROLL) {
                raw_velocity_roll_ = vel_raw; raw_position_roll_ = pos_raw;
            } else if (id == DXL_ID_PITCH) {
                raw_velocity_pitch_ = vel_raw; raw_position_pitch_ = pos_raw;
            } else if (id == DXL_ID_YAW) {
                raw_velocity_yaw_ = vel_raw; raw_position_yaw_ = pos_raw;
            }
        }
        return got_all_data;
    }

    void update_dcms(double phi, double theta, double psi) {
        double cphi = cos(phi), sphi = sin(phi);
        double cth = cos(theta), sth = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);

        // C_J/LPR = RotX(phi) - Assuming J=LRB frame and roll is about X axis
        C_J_LPR_ << 1.0,  0.0,   0.0,
                   0.0,  cphi,  sphi,
                   0.0, -sphi,  cphi;

        // C_J/LYR from PDF Eq 18b
        C_J_LYR_ << cth,  sphi*sth,  cphi*sth,
                   0.0,  cphi,     -sphi,
                  -sth,  sphi*cth,  cphi*cth;

        // C_J/I from PDF Eq 18c
        C_J_I_ << cth*cpsi,                      cth*spsi,                     -sth,
                 sphi*sth*cpsi - cphi*spsi,    sphi*sth*spsi + cphi*cpsi,   sphi*cth,
                 cphi*sth*cpsi + sphi*spsi,    cphi*sth*spsi - sphi*cpsi,   cphi*cth;
    }

     Eigen::Vector3d compute_omega_LYR_J(double phi_dot, double theta_dot) {
         // Based on PDF Eq 24 (W_LYR/J = - W_J/LYR)
         // W_J/LYR = W_J/LRB(0) + W_LRB/LPR + W_LPR/LYR
         Eigen::Vector3d omega_LRB_LPR_in_J(phi_dot, 0, 0); // In J=LRB frame (Roll about X) <<< VERIFY AXIS
         Eigen::Vector3d omega_LPR_LYR_in_LPR(0, theta_dot, 0); // In LPR frame (Pitch about Y) <<< VERIFY AXIS
         // Rotate omega_LPR_LYR from LPR frame to J frame using C_J/LPR
         Eigen::Vector3d omega_LPR_LYR_in_J = C_J_LPR_ * omega_LPR_LYR_in_LPR;
         Eigen::Vector3d omega_J_LYR = omega_LRB_LPR_in_J + omega_LPR_LYR_in_J;
         return -omega_J_LYR;
     }

    // <<< NEEDS PROPER DERIVATION based on Eq 25 / Transport Theorem >>>
    Eigen::Vector3d compute_omega_dot_LYR_J(double phi_dot, double theta_dot, double phi_ddot, double theta_ddot) {
         Eigen::Vector3d omega_dot_LRB_LPR_J(phi_ddot, 0, 0); // <<< VERIFY AXIS
         Eigen::Vector3d omega_dot_LPR_LYR_LPR(0, theta_ddot, 0); // <<< VERIFY AXIS
         // Approximation - Ignores transport theorem terms involving C_dot or omega_cross
         Eigen::Vector3d omega_dot_LPR_LYR_J_approx = C_J_LPR_ * omega_dot_LPR_LYR_LPR;
         Eigen::Vector3d omega_dot_J_LYR_approx = omega_dot_LRB_LPR_J + omega_dot_LPR_LYR_J_approx;
         RCLCPP_WARN_ONCE(this->get_logger(), "Using approximate calculation for omega_dot_LYR_J. Verify derivation!");
         return -omega_dot_J_LYR_approx; // Return negative based on Eq 25 relationship
    }

     Eigen::Matrix3d compute_inverse_jacobian(double phi, double theta) {
         // Gamma from PDF Eq 50
         double cphi = cos(phi), sphi = sin(phi);
         double cth = cos(theta);
         Eigen::Matrix3d Gamma = Eigen::Matrix3d::Identity(); // Default for safety near singularity
         if (abs(cth) < 1e-6) {
             RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal lock condition (theta near +/- 90deg)! Using Identity for Inv Jacobian.");
         } else {
             double tan_th = tan(theta);
             Gamma << 1.0, sphi * tan_th,  cphi * tan_th,
                      0.0, cphi,          -sphi,
                      0.0, sphi / cth,     cphi / cth;
         }
         return Gamma;
     }

    // Updated Torque to Current Conversion using constants from E-Manual
    int32_t torque_to_goal_current(double torque_nm) {
        if (abs(TORQUE_CONSTANT_APPROX) < 1e-9) { // Use smaller tolerance
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Torque constant is effectively zero!");
            return 0; // Prevent division by zero
        }
        double current_A = torque_nm / TORQUE_CONSTANT_APPROX;
        double steps = current_A / CURRENT_STEP;
        // Clamp based on default current limit from manual
        double max_steps = DEFAULT_CURRENT_LIMIT_STEPS;
        // Clamp using std::max/min for clarity
        steps = std::max(-max_steps, std::min(max_steps, steps));
        // Return as int32_t, compatible with SDK potentially needing signed values
        return static_cast<int32_t>(std::round(steps));
    }

    // SDK Write Helpers with improved logging/safety
    void write_byte_with_log(uint8_t dxl_id, uint16_t addr, uint8_t value, const std::string& op_name) {
        if (!portHandler_ || !packetHandler_) return; // Check if SDK objects are valid
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dxl_id, addr, value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "ID:%d Addr:%d (%s) Comm Fail: %s", dxl_id, addr, op_name.c_str(), packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "ID:%d Addr:%d (%s) DXL Error: %s", dxl_id, addr, op_name.c_str(), packetHandler_->getRxPacketError(dxl_error));
        }
    }
    void write1Byte(uint8_t dxl_id, uint16_t addr, uint8_t value, bool log_on_error = false) {
        if(log_on_error) write_byte_with_log(dxl_id, addr, value, "Write1Byte");
        else if (portHandler_ && packetHandler_) { // Check SDK objects before calling
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, addr, value, nullptr);
        }
    }

    // Send commands via SyncWrite
    bool send_dynamixel_commands(int32_t current_roll, int32_t current_pitch, int32_t current_yaw) {
        if (!groupSyncWrite_) return false; // Check if object is valid

        bool result_add = true;
        uint8_t param_goal_current[LEN_GOAL_CURRENT]; // Size = 2 bytes

        // Add Roll param (use static_cast<uint16_t> for SDK function)
        uint16_t roll_cmd_word = static_cast<uint16_t>(current_roll);
        param_goal_current[0] = DXL_LOBYTE(roll_cmd_word); param_goal_current[1] = DXL_HIBYTE(roll_cmd_word);
        if (!groupSyncWrite_->addParam(DXL_ID_ROLL, param_goal_current)) { result_add = false; }

        // Add Pitch param
        uint16_t pitch_cmd_word = static_cast<uint16_t>(current_pitch);
        param_goal_current[0] = DXL_LOBYTE(pitch_cmd_word); param_goal_current[1] = DXL_HIBYTE(pitch_cmd_word);
        if (!groupSyncWrite_->addParam(DXL_ID_PITCH, param_goal_current)) { result_add = false; }

        // Add Yaw param
        uint16_t yaw_cmd_word = static_cast<uint16_t>(current_yaw);
        param_goal_current[0] = DXL_LOBYTE(yaw_cmd_word); param_goal_current[1] = DXL_HIBYTE(yaw_cmd_word);
        if (!groupSyncWrite_->addParam(DXL_ID_YAW, param_goal_current)) { result_add = false; }

        if (!result_add) {
            RCLCPP_ERROR(this->get_logger(), "Failed to add one or more SyncWrite parameters.");
            groupSyncWrite_->clearParam(); // Clear potentially added params
            return false;
        }

        // Transmit SyncWrite packet
        int dxl_comm_result = groupSyncWrite_->txPacket();
        bool tx_success = (dxl_comm_result == COMM_SUCCESS);
        if (!tx_success) {
            RCLCPP_ERROR(this->get_logger(), "SyncWrite txPacket failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        }

        groupSyncWrite_->clearParam(); // Always clear params after attempt
        return tx_success;
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
       RCLCPP_FATAL(rclcpp::get_logger("Main"), "An unexpected error occurred during node creation/spin: %s", e.what());
  }

  // Ensure shutdown happens even if spin wasn't reached or exited early
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Shutting down node.");
  // node->~DroneDynamixelBridgeNode(); // Destructor called automatically by shared_ptr going out of scope
  rclcpp::shutdown();
  return 0;
}
