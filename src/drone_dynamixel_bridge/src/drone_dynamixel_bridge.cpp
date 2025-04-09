#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h> // Includes group_sync_write.h and group_bulk_read.h
#include <chrono>
#include <mutex>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense> // Ensure Eigen is linked

using namespace std::chrono_literals;

// --- Dynamixel Configuration ---
// IDs
const uint8_t DXL_ID_ROLL  = 2;
const uint8_t DXL_ID_PITCH = 1;
const uint8_t DXL_ID_YAW   = 3;
std::vector<uint8_t> DXL_IDS = {DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};
// Port & Baud
const char* DEVICENAME     = "/dev/ttyUSB1"; // Adjust port
const int   BAUDRATE       = 4000000;
const float PROTOCOL_VERSION = 2.0;
// Addresses (VERIFY FOR MX-106T)
const uint16_t ADDR_TORQUE_ENABLE    = 64;
const uint16_t ADDR_GOAL_CURRENT     = 102; // Assuming Current Control Mode
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t ADDR_PRESENT_POSITION = 132;
// Data Lengths
const uint16_t LEN_GOAL_CURRENT     = 2; // Often 2 bytes for current
const uint16_t LEN_PRESENT_VELOCITY = 4;
const uint16_t LEN_PRESENT_POSITION = 4;
// Conversion Factors (MUST VERIFY/CALIBRATE FOR MX-106T)
const double VELOCITY_TO_RAD_PER_SEC = 0.229 * (2.0 * M_PI / 60.0); // 0.229 RPM per unit -> rad/s
const double POSITION_TO_RAD         = (2.0 * M_PI / 4096.0);     // 4096 units per revolution -> rad
const double TORQUE_CONSTANT_APPROX  = 1.5; // N*m per Ampere (EXAMPLE - FIND REAL VALUE)
const double CURRENT_STEP            = 0.001; // Amperes per unit for Goal Current (EXAMPLE - FIND REAL VALUE, often 2.69mA or similar for X-series)


// --- Filter Differentiator Class (Unchanged) ---
class FilterDiff {
public:
    FilterDiff(double wn, double zeta) : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), initialized_(false) {}
    void update(double input, double dt) {
        if (dt <= 0) return;
        if (!initialized_) {
            x1_ = input; x2_ = 0.0; initialized_ = true;
        } else {
            double dx1 = x2_;
            double dx2 = -wn_ * wn_ * x1_ - 2.0 * zeta_ * wn_ * x2_ + wn_ * wn_ * input;
            x1_ += dx1 * dt; x2_ += dx2 * dt;
        }
    }
    double getFilteredValue() const { return x1_; }
    double getFilteredDerivative() const { return x2_; }
    void reset(double initial_value = 0.0) { x1_ = initial_value; x2_ = 0.0; initialized_ = true; }
private:
    double wn_, zeta_, x1_, x2_; bool initialized_;
};

// --- Theoretical Dynamics Equations Class (Unchanged from previous C++ version) ---
class EqDynamics {
public:
    EqDynamics() {}
    Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &v) { /* ... as before ... */ }
    Eigen::Matrix3d computeInertia(const Eigen::Matrix3d &IJ_tilde, const Eigen::Matrix3d &C_JLi) { /* ... as before ... */ }
    Eigen::Vector3d computeIDomegaDt(const Eigen::Matrix3d &IJ, const Eigen::Vector3d &omega_dot) { /* ... as before ... */ }
    Eigen::Vector3d computeOmegaCrossIomega(const Eigen::Matrix3d &IJ, const Eigen::Vector3d &omega1, const Eigen::Vector3d &omega2) { /* ... as before ... */ }
    Eigen::Vector3d computeIOmegaCrossOmega(const Eigen::Matrix3d &IJ, const Eigen::Vector3d &omega1, const Eigen::Vector3d &omega2) { /* ... as before ... */ }

    // IMPORTANT: This computeTorque calculates the sum of terms for *one* component Li.
    // It needs to be called for each component (RB, PR, YR) and the results summed.
    Eigen::Vector3d computeComponentInertialTorque(
        const Eigen::Matrix3d &I_Ci, // Inertia of component i relative to its CoM, in frame i
        const Eigen::Matrix3d &C_Ji, // Rotation matrix from component frame Li to base frame J
        const Eigen::Vector3d &omega_JI, // Absolute angular velocity
        const Eigen::Vector3d &omega_LiJ,// Relative angular velocity of Li wrt J, expressed in J
        const Eigen::Vector3d &omega_JI_dot, // Absolute angular acceleration
        const Eigen::Vector3d &omega_LiJ_dot // Relative angular acceleration
    ) {
        // Inertia tensor of component i expressed in frame J
        Eigen::Matrix3d IJ_i = C_Ji * I_Ci * C_Ji.transpose(); // Assuming I_Ci is given in Li frame aligned with CoM

        // Absolute angular velocity of component i: omega_LiI = omega_J/I + omega_Li/J
        Eigen::Vector3d omega_LiI = omega_JI + omega_LiJ;
        // Absolute angular acceleration of component i: d(omega_LiI)/dt (approx)
        // Note: A more rigorous derivation involves the transport theorem.
        // Using the simple sum of filtered derivatives as an approximation here.
        // Need d(omega_LiJ)/dt in frame J.
        Eigen::Vector3d omega_LiI_dot_approx = omega_JI_dot + omega_LiJ_dot; // Approximation, assumes J isn't accelerating significantly relative to I in a way that affects LiJ derivative

        // Based on general rigid body dynamics: Torque = I*alpha + omega x (I*omega)
        Eigen::Vector3d component_torque = computeIDomegaDt(IJ_i, omega_LiI_dot_approx) // Needs careful derivation check, this is T = I*alpha
                                          + computeOmegaCrossIomega(IJ_i, omega_LiI, omega_LiI); // This is omega x (I*omega)

        // --- Revisit based on PDF Derivations ---
        // The PDF (Eq 34, 39, 44) seems to use combinations of J/I and Li/J terms directly.
        // Let's try to replicate Eq 16 structure for component 'i'
        // T_i = I_i * d(omega_J/I)/dt + omega_J/I x (I_i*omega_J/I)   <- Absolute part
        //      + I_i * d(omega_Li/J)/dt + omega_J/I x (I_i*omega_Li/J) <- Interaction 1
        //      + omega_Li/J x (I_i*omega_J/I) + omega_Li/J x (I_i*omega_Li/J) <- Interaction 2&3

        Eigen::Vector3d term_abs_accel = computeIDomegaDt(IJ_i, omega_JI_dot);
        Eigen::Vector3d term_abs_gyro  = computeOmegaCrossIomega(IJ_i, omega_JI, omega_JI);
        Eigen::Vector3d term_rel_accel = computeIDomegaDt(IJ_i, omega_LiJ_dot); // Careful with frame of derivative
        Eigen::Vector3d term_interact1 = computeOmegaCrossIomega(IJ_i, omega_JI, omega_LiJ);
        Eigen::Vector3d term_interact2 = computeOmegaCrossIomega(IJ_i, omega_LiJ, omega_JI);
        Eigen::Vector3d term_rel_gyro  = computeOmegaCrossIomega(IJ_i, omega_LiJ, omega_LiJ);

        // Summing based on visual inspection of terms potentially matching Eq 16 applied per component
        component_torque = term_abs_accel + term_abs_gyro + term_rel_accel + term_interact1 + term_interact2 + term_rel_gyro;

        // The term involving derivatives of I (dIJ/dt) needs C_dot, related via Darboux Eq 13/14b
        // d(IJ_i)/dt = [omega_J/I]x * IJ_i - IJ_i * [omega_J/I]x  (If IJ_i is constant in J frame, which isn't true)
        // d(IJ_i)/dt = d(CJi*ICi*CJi^T)/dt = Cdot*ICi*CJiT + CJi*ICi*CdotT
        // Cdot = CJi*[omega_Li/J]x ?? No, Cdot = [omega_J/Li]x * CJi ?? Check PDF Eq 13/14b more carefully
        // Eq 14b: dI/dt|J = [omega_J/I]x * I - I * [omega_J/I]x  (This assumes I is constant in frame I, which I_Ci is NOT)
        // Eq 13/14b correct interpretation: Derivative of component inertia *in J frame*
        Eigen::Matrix3d IJ_i_dot = crossProductMatrix(omega_JI) * IJ_i - IJ_i * crossProductMatrix(omega_JI); // Simpler form from Eq 14b - CHECK VALIDITY
        Eigen::Vector3d term_I_dot = IJ_i_dot * omega_LiI; // This term might be part of T_i, compare Eq 4 vs Eq 16

        // For now, stick to the sum derived from individual terms visually matching Eq 16 structure
         return component_torque; // Return the inertial torque generated *by* this component's motion
    }
};


// --- ROS2 Node Class ---
class DroneDynamixelBridgeNode : public rclcpp::Node {
public:
  DroneDynamixelBridgeNode()
  : Node("drone_dynamixel_bridge_node"),
    // Initialize dynamics properties (VALUES FROM PDF TABLE 2 - Assumed wrt CoM, expressed in I initially?)
    // Need to decide how I_Ci (passed to computeComponentInertialTorque) is defined: WRT CoM in Li frame?
    // Assuming values from Table 2 are I_Ci IN INERTIAL FRAME I. We need them in Li frame or handle transformation.
    // For simplicity now, ASSUME values are I_Ci in Li frame (requires CAD export wrt component frame)
    I_CUAV_(Eigen::Matrix3d::Identity() * 0.01), // Placeholder - USE TABLE 2 VALUES CORRECTLY
    I_CRB_(Eigen::Matrix3d::Identity() * 0.01),  // Placeholder - USE TABLE 2 VALUES CORRECTLY
    I_CPR_(Eigen::Matrix3d::Identity() * 0.01),  // Placeholder - USE TABLE 2 VALUES CORRECTLY
    I_CYR_(Eigen::Matrix3d::Identity() * 0.01)   // Placeholder - USE TABLE 2 VALUES CORRECTLY
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Drone Dynamixel Bridge Node...");

    // --- Parameters ---
    // Using fixed values for now as requested, can be converted to parameters later
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
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixels. Shutting down.");
        // Use a clean shutdown mechanism if available in constructor context
        rclcpp::shutdown();
        throw std::runtime_error("Dynamixel initialization failed"); // Prevent node creation
    }

    // --- Initialize Dynamics Model ---
    dynamics_ = std::make_shared<EqDynamics>();

    // --- Initialize Filters ---
    // Absolute motion filters
    fd_roll_abs_  = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_pitch_abs_ = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_yaw_abs_   = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    // Relative motion filters (Using same gains for now)
    fd_roll_rel_  = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_pitch_rel_ = std::make_unique<FilterDiff>(filter_omega, filter_zeta);
    fd_yaw_rel_   = std::make_unique<FilterDiff>(filter_omega, filter_zeta);

    // --- Initialize Bulk Read ---
    groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
    setup_bulk_read();

    // --- Initialize Sync Write ---
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    prev_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "DroneDynamixelBridgeNode initialized successfully.");
  }

  ~DroneDynamixelBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down. Disabling Dynamixel torque.");
    // Disable torque safely
    for (uint8_t id : DXL_IDS) {
        write1Byte(id, ADDR_TORQUE_ENABLE, 0, true); // Force log on error during shutdown
    }
    if (portHandler_) {
        portHandler_->closePort();
    }
  }

private:
    // --- State Variables ---
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
    px4_msgs::msg::VehicleOdometry::SharedPtr latest_odometry_msg_;
    rclcpp::Time prev_time_;
    bool odometry_received_ = false;

    // Raw Dynamixel Readings
    int32_t raw_velocity_roll_ = 0;
    int32_t raw_velocity_pitch_ = 0;
    int32_t raw_velocity_yaw_ = 0;
    uint32_t raw_position_roll_ = 0;
    uint32_t raw_position_pitch_ = 0;
    uint32_t raw_position_yaw_ = 0;

    // Processed Angles (Relative - rad)
    double current_phi_ = 0.0;   // Roll angle (relative)
    double current_theta_ = 0.0; // Pitch angle (relative)
    double current_psi_ = 0.0;   // Yaw angle (relative)

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
    const Eigen::Matrix3d I_CUAV_; // Inertia Tensors (relative to CoM, in component frame?)
    const Eigen::Matrix3d I_CRB_;
    const Eigen::Matrix3d I_CPR_;
    const Eigen::Matrix3d I_CYR_;
    Eigen::Matrix3d C_J_LRB_ = Eigen::Matrix3d::Identity(); // Constant Identity
    Eigen::Matrix3d C_J_LPR_; // Rotation J <- LPR (depends on phi)
    Eigen::Matrix3d C_J_LYR_; // Rotation J <- LYR (depends on phi, theta)
    Eigen::Matrix3d C_J_I_;   // Rotation J <- I   (depends on phi, theta, psi)

    // --- Callbacks ---
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odometry_msg_ = msg;
        odometry_received_ = true;
        // Potentially update C_J_I_ here if needed, but the required DCMs for dynamics
        // depend on relative angles phi, theta, psi read from Dynamixels.
    }

    // --- Timer Callback (Main Loop) ---
    void timer_callback() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 0.0 || dt > 0.1) { // Check for valid timestep
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestep dt = %.4f. Skipping update.", dt);
             prev_time_ = current_time;
             // Consider resetting filters if dt is too large
             return;
        }
        prev_time_ = current_time;

        // --- 1. Read Dynamixel Data (Velocity & Position) ---
        if (!read_dynamixel_data()) {
            // Error logged in helper function, skip cycle
            return;
        }

        // --- 2. Process Dynamixel Readings ---
        // Convert raw velocity to rad/s
        double phi_dot_raw   = static_cast<double>(raw_velocity_roll_) * VELOCITY_TO_RAD_PER_SEC;
        double theta_dot_raw = static_cast<double>(raw_velocity_pitch_) * VELOCITY_TO_RAD_PER_SEC;
        double psi_dot_raw   = static_cast<double>(raw_velocity_yaw_) * VELOCITY_TO_RAD_PER_SEC;
        // Convert raw position to rad (assuming 0 is aligned)
        current_phi_   = static_cast<double>(raw_position_roll_) * POSITION_TO_RAD; // Adjust zero offset if needed
        current_theta_ = static_cast<double>(raw_position_pitch_) * POSITION_TO_RAD;
        current_psi_   = static_cast<double>(raw_position_yaw_) * POSITION_TO_RAD;

        // --- 3. Get Absolute Velocity from Odometry ---
        px4_msgs::msg::VehicleOdometry::SharedPtr odom;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!odometry_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Odometry...");
                return; // Need odometry
            }
            odom = latest_odometry_msg_;
        }
        Eigen::Vector3d omega_JI_raw(odom->angular_velocity[0],
                                     odom->angular_velocity[1],
                                     odom->angular_velocity[2]);

        // --- 4. Filter Velocities to get Accelerations ---
        // Absolute
        fd_roll_abs_->update(omega_JI_raw.x(), dt);
        fd_pitch_abs_->update(omega_JI_raw.y(), dt);
        fd_yaw_abs_->update(omega_JI_raw.z(), dt);
        Eigen::Vector3d omega_JI(fd_roll_abs_->getFilteredValue(),
                                 fd_pitch_abs_->getFilteredValue(),
                                 fd_yaw_abs_->getFilteredValue());
        Eigen::Vector3d omega_JI_dot(fd_roll_abs_->getFilteredDerivative(),
                                     fd_pitch_abs_->getFilteredDerivative(),
                                     fd_yaw_abs_->getFilteredDerivative());
        // Relative (using scalar velocities corresponding to phi_dot, theta_dot, psi_dot)
        fd_roll_rel_->update(phi_dot_raw, dt);
        fd_pitch_rel_->update(theta_dot_raw, dt);
        fd_yaw_rel_->update(psi_dot_raw, dt);
        double phi_dot_filt   = fd_roll_rel_->getFilteredValue();
        double theta_dot_filt = fd_pitch_rel_->getFilteredValue();
        double psi_dot_filt   = fd_yaw_rel_->getFilteredValue();
        double phi_ddot_filt  = fd_roll_rel_->getFilteredDerivative();
        double theta_ddot_filt= fd_pitch_rel_->getFilteredDerivative();
        double psi_ddot_filt  = fd_yaw_rel_->getFilteredDerivative();


        // --- 5. Calculate Relative Velocity & Acceleration Vectors (omega_Li/J, omega_dot_Li/J) ---
        // Based on PDF Section 5 (Eq 20-25), construct vectors in frame J
        Eigen::Vector3d omega_LRB_J(0, 0, 0); // omega_LRB/J = -omega_J/LRB = 0 (Eq 21b)
        Eigen::Vector3d omega_LPR_J(-phi_dot_filt, 0, 0); // Eq 22 (Assuming phi_dot is from roll motor)
        Eigen::Vector3d omega_LYR_J = compute_omega_LYR_J(phi_dot_filt, theta_dot_filt); // Needs helper based on Eq 24

        Eigen::Vector3d omega_dot_LRB_J(0, 0, 0); // Eq 21c
        Eigen::Vector3d omega_dot_LPR_J(-phi_ddot_filt, 0, 0); // Eq 23 (Derivative of Eq 22)
        Eigen::Vector3d omega_dot_LYR_J = compute_omega_dot_LYR_J(phi_dot_filt, theta_dot_filt, phi_ddot_filt, theta_ddot_filt); // Needs helper based on Eq 25


        // --- 6. Calculate Orientation Matrices (DCMs) ---
        update_dcms(current_phi_, current_theta_, current_psi_); // Update C_J_LPR_, C_J_LYR_, C_J_I_

        // --- 7. Calculate Component Inertial Torques ---
        // UAV Term (simplified using Eq 29)
        Eigen::Vector3d T_UAV_inertia = dynamics_->computeIDomegaDt(I_CUAV_, omega_JI_dot) // Assuming I_CUAV is in J=UAV frame
                                       + dynamics_->computeOmegaCrossIomega(I_CUAV_, omega_JI, omega_JI);

        // Roll Bar Term (Eq 34, simplified using Eq 30-33)
        Eigen::Vector3d T_RB = dynamics_->computeIDomegaDt(I_CRB_, omega_JI_dot) // Assuming I_CRB in RB frame = J frame
                              + dynamics_->computeOmegaCrossIomega(I_CRB_, omega_JI, omega_JI);

        // Pitch Ring Term (Eq 39 structure, using computeComponentInertialTorque)
        // Need omega_LPR/J and its derivative correctly
        Eigen::Vector3d T_PR = dynamics_->computeComponentInertialTorque(I_CPR_, C_J_LPR_, omega_JI, omega_LPR_J, omega_JI_dot, omega_dot_LPR_J);

        // Yaw Ring Term (Eq 44 structure, using computeComponentInertialTorque)
        // Need omega_LYR/J and its derivative correctly
        Eigen::Vector3d T_YR = dynamics_->computeComponentInertialTorque(I_CYR_, C_J_LYR_, omega_JI, omega_LYR_J, omega_JI_dot, omega_dot_LYR_J);

        // --- 8. Calculate Total Cancellation Torque ---
        Eigen::Vector3d total_inertial_torque = T_UAV_inertia + T_RB + T_PR + T_YR;
        // Add T_UAV offset term if applicable (Eq 52) - requires COM offset 'p' and forces Ti
        // total_inertial_torque -= p_vector.cross(Total_Thrust_Vector); // Placeholder

        Eigen::Vector3d required_motor_torque = -total_inertial_torque; // Apply negative to cancel

        // --- 9. Convert Torque Vector to Individual Motor Torques (Mx, My, Mz) ---
        // Use Inverse Jacobian Gamma (Eq 50)
        Eigen::Matrix3d Gamma = compute_inverse_jacobian(current_phi_, current_theta_);
        Eigen::Vector3d gimbal_torques = Gamma * required_motor_torque; // Mx, My, Mz

        double roll_motor_torque_cmd  = gimbal_torques.x(); // Mx
        double pitch_motor_torque_cmd = gimbal_torques.y(); // My
        double yaw_motor_torque_cmd   = gimbal_torques.z(); // Mz

        // --- 10. Convert Nm to Dynamixel Goal Current ---
        // Apply torque limits if known?
        int32_t goal_current_roll  = torque_to_goal_current(roll_motor_torque_cmd);
        int32_t goal_current_pitch = torque_to_goal_current(pitch_motor_torque_cmd);
        int32_t goal_current_yaw   = torque_to_goal_current(yaw_motor_torque_cmd);

        // --- 11. Send Commands via SyncWrite ---
        if (!send_dynamixel_commands(goal_current_roll, goal_current_pitch, goal_current_yaw)) {
            // Error logged in helper
        }

        // --- Logging ---
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "ReqTorque: [%.3f,%.3f,%.3f] | CmdCurr: [%d,%d,%d]",
          required_motor_torque.x(), required_motor_torque.y(), required_motor_torque.z(),
          goal_current_roll, goal_current_pitch, goal_current_yaw);

    } // End timer_callback

    // --- Helper Functions ---

    bool initialize_dynamixels() { /* ... as before, ensure torque enable ... */ }

    void setup_bulk_read() {
        // Add params for reading VELOCITY and POSITION for all 3 motors
        // Read 8 bytes (Vel + Pos) starting from ADDR_PRESENT_VELOCITY
        uint16_t read_start_addr = ADDR_PRESENT_VELOCITY;
        uint16_t read_len = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;
        bool result = false;
        for(uint8_t id : DXL_IDS) {
            result = groupBulkRead_->addParam(id, read_start_addr, read_len);
            if (!result) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add BulkRead param for ID %d", id);
            }
        }
        if (!result) {
             RCLCPP_ERROR(this->get_logger(), "Failed setting up BulkRead parameters.");
             // Consider throwing or handling this failure
        }
    }

    bool read_dynamixel_data() {
        int dxl_comm_result = groupBulkRead_->txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "BulkRead failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
            return false;
        }

        bool got_roll = false, got_pitch = false, got_yaw = false;
        uint16_t read_len = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

        // Check and get data for Roll motor
        got_roll = groupBulkRead_->isAvailable(DXL_ID_ROLL, ADDR_PRESENT_VELOCITY, read_len);
        if (got_roll) {
            raw_velocity_roll_ = groupBulkRead_->getData(DXL_ID_ROLL, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            raw_position_roll_ = groupBulkRead_->getData(DXL_ID_ROLL, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        } else { RCLCPP_ERROR(this->get_logger(), "BulkRead data not available for Roll ID %d", DXL_ID_ROLL); }

        // Check and get data for Pitch motor
        got_pitch = groupBulkRead_->isAvailable(DXL_ID_PITCH, ADDR_PRESENT_VELOCITY, read_len);
        if (got_pitch) {
            raw_velocity_pitch_ = groupBulkRead_->getData(DXL_ID_PITCH, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            raw_position_pitch_ = groupBulkRead_->getData(DXL_ID_PITCH, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        } else { RCLCPP_ERROR(this->get_logger(), "BulkRead data not available for Pitch ID %d", DXL_ID_PITCH); }

        // Check and get data for Yaw motor
        got_yaw = groupBulkRead_->isAvailable(DXL_ID_YAW, ADDR_PRESENT_VELOCITY, read_len);
        if (got_yaw) {
            raw_velocity_yaw_ = groupBulkRead_->getData(DXL_ID_YAW, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            raw_position_yaw_ = groupBulkRead_->getData(DXL_ID_YAW, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        } else { RCLCPP_ERROR(this->get_logger(), "BulkRead data not available for Yaw ID %d", DXL_ID_YAW); }

        return got_roll && got_pitch && got_yaw; // Return true only if all data was received
    }

    void update_dcms(double phi, double theta, double psi) {
        double cphi = cos(phi), sphi = sin(phi);
        double cth = cos(theta), sth = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);

        // C_J_LPR (Eq 18a, transpose of C_LPR_J) -> Rotation depends only on Roll angle phi
        C_J_LPR_ << 1.0, 0.0,   0.0,
                   0.0, cphi, -sphi,
                   0.0, sphi,  cphi;

        // C_J_LYR (Eq 18b, transpose of C_LYR_J) -> Depends on Roll (phi) and Pitch (theta)
        C_J_LYR_ << cth,  sphi*sth,  cphi*sth,
                   0.0,  cphi,     -sphi,
                  -sth,  sphi*cth,  cphi*cth;

        // C_J_I (Eq 18c, transpose of C_I_J) -> Depends on Roll, Pitch, Yaw (phi, theta, psi)
        C_J_I_ << cth*cpsi,                      cth*spsi,                     -sth,
                 sphi*sth*cpsi - cphi*spsi,    sphi*sth*spsi + cphi*cpsi,   sphi*cth,
                 cphi*sth*cpsi + sphi*spsi,    cphi*sth*spsi - sphi*cpsi,   cphi*cth;
    }

     Eigen::Vector3d compute_omega_LYR_J(double phi_dot, double theta_dot) {
         // Reconstruct WJ/LYR from Eq 24, then negate
         // WJ/LYR = WJ/LRB + WLRB/LPR + WLPR/LYR
         // WJ/LRB = 0
         // WLRB/LPR = [phi_dot, 0, 0]^T (in J=LRB frame)
         // WLPR/LYR = C_LPR/LYR * [0, theta_dot, 0]^T (in LPR frame)
         // Need C_J/LPR and C_LPR/LYR from Eq 17b, 17c
         Eigen::Matrix3d C_LRB_LPR; // Eq 17b
         C_LRB_LPR << 1, 0, 0, 0, cos(current_phi_), sin(current_phi_), 0, -sin(current_phi_), cos(current_phi_);
         Eigen::Matrix3d C_LPR_LYR; // Eq 17c
         C_LPR_LYR << cos(current_theta_), 0, -sin(current_theta_), 0, 1, 0, sin(current_theta_), 0, cos(current_theta_);

         Eigen::Vector3d omega_LRB_LPR_J(phi_dot, 0, 0); // Already in J frame
         Eigen::Vector3d omega_LPR_LYR_LPR(0, theta_dot, 0); // In LPR frame
         // Rotate omega_LPR_LYR to J frame: C_J/LPR * omega_LPR_LYR_LPR
         Eigen::Vector3d omega_LPR_LYR_J = C_J_LPR_ * omega_LPR_LYR_LPR;

         Eigen::Vector3d omega_J_LYR = omega_LRB_LPR_J + omega_LPR_LYR_J;
         return -omega_J_LYR;
     }

    Eigen::Vector3d compute_omega_dot_LYR_J(double phi_dot, double theta_dot, double phi_ddot, double theta_ddot) {
        // Based on Eq 25, requires careful application of derivatives and Theorem 1
        // This is complex and error-prone to write directly.
        // For now, provide a placeholder (zero or simple derivative of velocity vector)
        // Placeholder: Simple numerical differentiation of the calculated omega_LYR_J vector (less accurate than filtered)
        // OR just use the filtered components directly as an approximation?
        // Eq 25 seems to be WJ/LYR_dot, so we need the negative?

        // Approximation: Rotate individual accelerations into J frame
         Eigen::Vector3d omega_dot_LRB_LPR_J(phi_ddot, 0, 0); // Already in J frame
         Eigen::Vector3d omega_dot_LPR_LYR_LPR(0, theta_ddot, 0); // In LPR frame
         // Rotate omega_dot_LPR_LYR to J frame: C_J/LPR * omega_dot_LPR_LYR_LPR
         // This ignores the C_dot term from Theorem 1 - INACCURATE but simpler placeholder
         Eigen::Vector3d omega_dot_LPR_LYR_J_approx = C_J_LPR_ * omega_dot_LPR_LYR_LPR;

         Eigen::Vector3d omega_dot_J_LYR_approx = omega_dot_LRB_LPR_J + omega_dot_LPR_LYR_J_approx;
         return -omega_dot_J_LYR_approx; // Placeholder - NEEDS PROPER DERIVATION
    }

     Eigen::Matrix3d compute_inverse_jacobian(double phi, double theta) {
         // Gamma from Eq 50
         double cphi = cos(phi), sphi = sin(phi);
         double cth = cos(theta), sth = sin(theta);
         Eigen::Matrix3d Gamma = Eigen::Matrix3d::Identity(); // Default
         if (abs(cth) > 1e-6) { // Avoid division by zero
             Gamma << 1.0, sphi * sth / cth, cphi * sth / cth,
                      0.0, cphi,             -sphi,
                      0.0, sphi / cth,       cphi / cth;
         } else {
             RCLCPP_ERROR(this->get_logger(), "Gimbal lock condition (theta near +/- 90deg)! Inverse Jacobian undefined.");
             // Handle gimbal lock case - maybe return identity or previous value?
         }
         return Gamma;
     }

    int32_t torque_to_goal_current(double torque_nm) { /* ... as before ... */ }
    void write1Byte(uint8_t id, uint16_t addr, uint8_t value, bool log_on_error = false) { /* ... improved error logging ... */ }
    void write2Byte(uint8_t id, uint16_t addr, uint16_t value, bool log_on_error = false) { /* ... improved error logging ... */ }

    bool send_dynamixel_commands(int32_t current_roll, int32_t current_pitch, int32_t current_yaw) {
        bool result = false;
        uint8_t param_goal_current[2]; // Goal Current is often 2 bytes

        // --- Roll ---
        // Convert int32_t signed current value to uint8_t[2] for SyncWrite
        // Dynamixel SDK handles signedness via two's complement for word writes usually.
        // We pass the target value as uint16_t, casting the possibly negative int32_t.
        uint16_t roll_cmd_word = static_cast<uint16_t>(current_roll);
        param_goal_current[0] = DXL_LOBYTE(roll_cmd_word);
        param_goal_current[1] = DXL_HIBYTE(roll_cmd_word);
        result = groupSyncWrite_->addParam(DXL_ID_ROLL, param_goal_current);
        if (!result) { RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWrite param for Roll ID %d", DXL_ID_ROLL); return false; }

        // --- Pitch ---
        uint16_t pitch_cmd_word = static_cast<uint16_t>(current_pitch);
        param_goal_current[0] = DXL_LOBYTE(pitch_cmd_word);
        param_goal_current[1] = DXL_HIBYTE(pitch_cmd_word);
        result = groupSyncWrite_->addParam(DXL_ID_PITCH, param_goal_current);
         if (!result) { RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWrite param for Pitch ID %d", DXL_ID_PITCH); return false; }

        // --- Yaw ---
        uint16_t yaw_cmd_word = static_cast<uint16_t>(current_yaw);
        param_goal_current[0] = DXL_LOBYTE(yaw_cmd_word);
        param_goal_current[1] = DXL_HIBYTE(yaw_cmd_word);
        result = groupSyncWrite_->addParam(DXL_ID_YAW, param_goal_current);
         if (!result) { RCLCPP_ERROR(this->get_logger(), "Failed to add SyncWrite param for Yaw ID %d", DXL_ID_YAW); return false; }

        // --- Transmit ---
        int dxl_comm_result = groupSyncWrite_->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "SyncWrite failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
            groupSyncWrite_->clearParam(); // Clear params after failure
            return false;
        }

        groupSyncWrite_->clearParam(); // Clear params after successful write
        return true;
    }

}; // End class DroneDynamixelBridgeNode

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
      auto node = std::make_shared<DroneDynamixelBridgeNode>();
      rclcpp::spin(node);
  } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node instantiation failed: %s", e.what());
      // Ensure Dynamixel port is closed if partially opened?
  }
  rclcpp::shutdown();
  return 0;
}
