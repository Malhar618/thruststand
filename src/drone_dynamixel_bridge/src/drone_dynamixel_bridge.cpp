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
#include <thread>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
//                         Dynamixel Configuration (write‑only)
// -----------------------------------------------------------------------------
const uint8_t  DXL_ID_ROLL   = 2;
const uint8_t  DXL_ID_PITCH  = 1;
const uint8_t  DXL_ID_YAW    = 3;
const std::vector<uint8_t> DXL_IDS{DXL_ID_ROLL, DXL_ID_PITCH, DXL_ID_YAW};

const char*   DEVICENAME     = "/dev/ttyUSB1";      //  <‑‑ verify on Odroid
const int     BAUDRATE       = 4'000'000;
const float   PROTOCOL_VER   = 2.0f;

const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_GOAL_CURRENT  = 102;
const uint16_t LEN_TORQUE_ENABLE  = 1;
const uint16_t LEN_GOAL_CURRENT   = 2;

const double   CURRENT_LSB        = 0.0045;   // 4.5 mA per tick
const int      CURRENT_OFFSET_RAW = 2048;     // register value @ 0 A
const int      CURRENT_RAW_MIN    = 0;
const int      CURRENT_RAW_MAX    = 4095;     // ±9.2115 A

// -----------------------------------------------------------------------------
//                               Helper classes
// -----------------------------------------------------------------------------
class FilterDiff {
public:
    FilterDiff(double wn, double zeta)
        : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0), init_(false) {}

    void update(double input, double dt) {
        if (dt <= 1e-9) return;
        if (!init_) {
            x1_ = input;
            x2_ = 0.0;
            init_ = true;
        } else {
            const double x1_dot = x2_;
            const double x2_dot = -wn_*wn_*x1_ - 2.0*zeta_*wn_*x2_ + wn_*wn_*input;
            x1_ += x1_dot * dt;
            x2_ += x2_dot * dt;
        }
    }

    double derivative() const { return x2_; }
    void reset(double v0 = 0.0) { x1_=v0; x2_=0.0; init_=true; }

private:
    double wn_, zeta_;
    double x1_, x2_;
    bool   init_;
};

class EqDynamics {
public:
    Eigen::Vector3d compute_torque(const Eigen::Matrix3d& I_li_li,
                                   const Eigen::Matrix3d& C_j_li,
                                   const Eigen::Vector3d& omega_raw_li,
                                   const Eigen::Vector3d& omega_dot_filt_li) const {
        const Eigen::Vector3d term1 = I_li_li * omega_dot_filt_li;
        const Eigen::Vector3d term2 = omega_raw_li.cross(I_li_li * omega_raw_li);
        return C_j_li * (term1 + term2);
    }
};

// -----------------------------------------------------------------------------
//                               Main ROS 2 Node
// -----------------------------------------------------------------------------
class DroneDxlBridge : public rclcpp::Node {
public:
    DroneDxlBridge()
        : Node("drone_dynamixel_bridge_node"),
          I_CUAV_((Eigen::Matrix3d() <<  0.012576,-0.000001,-0.000069,
                                        -0.000001, 0.019215, 0.000012,
                                        -0.000069, 0.000012, 0.026610).finished()),
          I_CRB_ ((Eigen::Matrix3d() <<  0.000866,-0.000001,-0.000068,
                                        -0.000001, 0.101389, 0.000004,
                                        -0.000068, 0.000004, 0.101225).finished()),
          I_CPR_ ((Eigen::Matrix3d() <<  0.640302, 0.000043, 0.000078,
                                         0.000043, 0.570491,-0.000025,
                                         0.000078,-0.000025, 1.200050).finished()),
          I_CYR_ ((Eigen::Matrix3d() <<  2.384163,-0.000049,-0.000104,
                                        -0.000049, 1.110062, 0.000781,
                                        -0.000104, 0.000781, 1.286313).finished())
    {
        RCLCPP_INFO(get_logger(), "🔧  Initialising node …");

        init_csv();
        init_filters();
        init_sdk();
        init_ros();

        RCLCPP_INFO(get_logger(), "Node ready.");
    }

    ~DroneDxlBridge() override {
        RCLCPP_INFO(get_logger(), "⏻  Shutting down …");
        disable_torque_all();
        if (port_) port_->closePort();
        if (csv_.is_open()) csv_.close();
        RCLCPP_INFO(get_logger(), "✔  Shutdown complete.");
    }

private:
    // -----------------------  Initialisation helpers  ----------------------- //
    void init_csv() {
        const auto now      = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << "diag_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << ".csv";
        csv_name_ = oss.str();
        csv_.open(csv_name_, std::ios::out);
        if (!csv_.is_open()) {
            RCLCPP_ERROR(get_logger(), "CSV open failed: %s", csv_name_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Logging → %s", csv_name_.c_str());
            csv_ << "stamp,phi,theta,psi,"      // angles
                 "p,q,r,p_dot,q_dot,r_dot,"      // body rates & accels
                 "Tix,Tiy,Tiz,Gx,Gy,Gz,"         // inertial & gimbal torques
                 "cmdTroll,cmdTpitch,cmdTyaw,"   // scaled cmd torque
                 "rawIroll,rawIpitch,rawIyaw\n"; // raw current cmd
            csv_.flush();
        }
    }

    void init_filters() {
        fd_p_ = std::make_unique<FilterDiff>(70.0, 0.7);
        fd_q_ = std::make_unique<FilterDiff>(70.0, 0.7);
        fd_r_ = std::make_unique<FilterDiff>(70.0, 0.7);
    }

    void init_sdk() {
        port_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VER);
        if (!port_ || !packet_) throw std::runtime_error("DXL SDK handler error");
        if (!port_->openPort())           throw std::runtime_error("DXL openPort failed");
        if (!port_->setBaudRate(BAUDRATE)) throw std::runtime_error("DXL setBaudRate failed");
        RCLCPP_INFO(get_logger(), "DXL port %s @ %d", DEVICENAME, BAUDRATE);
        for (uint8_t id : DXL_IDS) write_byte(id, ADDR_TORQUE_ENABLE, 1, "torque enable");
        group_write_ = std::make_unique<dynamixel::GroupSyncWrite>(port_, packet_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);
    }

    void init_ros() {
        using px4_msgs::msg::VehicleOdometry;
        odom_sub_ = create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [&](VehicleOdometry::SharedPtr msg){ std::lock_guard<std::mutex> lk(mu_); odom_ = std::move(msg); have_odom_=true; });
        timer_ = create_wall_timer(5ms, [&]{ timer_cb(); });
        prev_time_ = now();
    }

    // ------------------------------  Callbacks  ----------------------------- //
    void timer_cb() {
        const rclcpp::Time t_now = now();
        double dt = (t_now - prev_time_).seconds();
        if (dt <= 1e-6 || dt > 0.1) return;     // skip if first or bad step
        prev_time_ = t_now;

        px4_msgs::msg::VehicleOdometry::SharedPtr od;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (!have_odom_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for odom …");
                return;
            }
            od = odom_;
        }

        // ---- raw body rates ----
        p_raw_ = od->angular_velocity[0];
        q_raw_ = od->angular_velocity[1];
        r_raw_ = od->angular_velocity[2];

        // ---- attitude ----
        Eigen::Quaterniond q_I_J(od->q[0], od->q[1], od->q[2], od->q[3]);
        q_I_J.normalize();
        C_J_I_ = q_I_J.toRotationMatrix().transpose();
        extract_euler(C_J_I_, phi_, theta_, psi_);

        // ---- filtered accel ----
        if (first_) {
            fd_p_->reset(p_raw_);
            fd_q_->reset(q_raw_);
            fd_r_->reset(r_raw_);
            first_ = false;
            return;   // skip first loop after reset
        }
        fd_p_->update(p_raw_, dt);
        fd_q_->update(q_raw_, dt);
        fd_r_->update(r_raw_, dt);
        p_dot_ = fd_p_->derivative();
        q_dot_ = fd_q_->derivative();
        r_dot_ = fd_r_->derivative();

        Eigen::Vector3d omega_JI(p_raw_, q_raw_, r_raw_);
        Eigen::Vector3d omega_dot_JI(p_dot_, q_dot_, r_dot_);

        // ---- Euler‑rate helper (only need phi̇, thetȧ for gimbal kinematics) ----
        euler_rates(phi_, theta_, p_raw_, q_raw_, r_raw_, phi_dot_, theta_dot_, psi_dot_);
        update_dcms(phi_, theta_);

        // ---- component angular velocities ----
        const Eigen::Vector3d omega_LRB_J(0,0,0);
        const Eigen::Vector3d omega_LPR_J(phi_dot_, 0, 0);
        const Eigen::Vector3d omega_LPR_LYR_in_LPR(0, theta_dot_, 0);
        const Eigen::Vector3d omega_LPR_LYR_in_J = C_J_LPR_ * omega_LPR_LYR_in_LPR;
        const Eigen::Vector3d omega_LYR_J = -(omega_LRB_J + omega_LPR_J + omega_LPR_LYR_in_J);

        const Eigen::Vector3d omega_dot_LRB_J(0,0,0);
        const Eigen::Vector3d omega_dot_LPR_J(0,0,0);
        const Eigen::Vector3d omega_dot_LYR_J(0,0,0);

        // ---- compose inertial-frame quantities ----
        const Eigen::Vector3d omega_UAV_J   = omega_JI;
        const Eigen::Vector3d omega_RB_J    = omega_LRB_J + omega_JI;
        const Eigen::Vector3d omega_PR_J    = omega_LPR_J + omega_JI;
        const Eigen::Vector3d omega_YR_J    = omega_LYR_J + omega_JI;

        const Eigen::Vector3d tt_PR  = omega_JI.cross(omega_LPR_J);
        const Eigen::Vector3d tt_YR  = omega_JI.cross(omega_LYR_J);

        const Eigen::Vector3d omega_dot_UAV_J = omega_dot_JI;
        const Eigen::Vector3d omega_dot_RB_J  = omega_dot_LRB_J + omega_dot_JI;
        const Eigen::Vector3d omega_dot_PR_J  = omega_dot_LPR_J + omega_dot_JI + tt_PR;
        const Eigen::Vector3d omega_dot_YR_J  = omega_dot_LYR_J + omega_dot_JI + tt_YR;

        // ---- transform to Li frames ----
        const Eigen::Matrix3d C_LPR_J = C_J_LPR_.transpose();
        const Eigen::Matrix3d C_LYR_J = C_J_LYR_.transpose();

        const Eigen::Vector3d omega_UAV_L = omega_UAV_J;
        const Eigen::Vector3d omega_RB_L  = omega_RB_J;
        const Eigen::Vector3d omega_PR_L  = C_LPR_J * omega_PR_J;
        const Eigen::Vector3d omega_YR_L  = C_LYR_J * omega_YR_J;

        const Eigen::Vector3d omega_dot_UAV_L = omega_dot_UAV_J;
        const Eigen::Vector3d omega_dot_RB_L  = omega_dot_RB_J;
        const Eigen::Vector3d omega_dot_PR_L  = C_LPR_J * omega_dot_PR_J;
        const Eigen::Vector3d omega_dot_YR_L  = C_LYR_J * omega_dot_YR_J;

        // ---- torques ----
        const Eigen::Vector3d T_RB = dyn_.compute_torque(I_CRB_, Eigen::Matrix3d::Identity(), omega_RB_L, omega_dot_RB_L);
        const Eigen::Vector3d T_PR = dyn_.compute_torque(I_CPR_, C_J_LPR_,            omega_PR_L, omega_dot_PR_L);
        const Eigen::Vector3d T_YR = dyn_.compute_torque(I_CYR_, C_J_LYR_,            omega_YR_L, omega_dot_YR_L);
        const Eigen::Vector3d total_T = T_RB + T_PR + T_YR;

        // ---- map to gimbal torques & current ----
        const Eigen::Vector3d gimbal_T = inv_jacobian(phi_, theta_) * total_T;
        const double roll_T  = 0.1 * gimbal_T.x();  // scale per your test
        const double pitch_T = 0.0;
        const double yaw_T   = 0.0;

        const int32_t roll_I  = torque_to_current(roll_T);
        const int32_t pitch_I = torque_to_current(pitch_T);
        const int32_t yaw_I   = torque_to_current(yaw_T);

        // ---- send ----
        send_currents(roll_I, pitch_I, yaw_I);

        // ---- log ----
        if (csv_.is_open()) {
            csv_ << std::fixed << std::setprecision(6)
                 << t_now.seconds() << ','
                 << phi_ << ',' << theta_ << ',' << psi_ << ','
                 << p_raw_ << ',' << q_raw_ << ',' << r_raw_ << ','
                 << p_dot_ << ',' << q_dot_ << ',' << r_dot_ << ','
                 << total_T.x() << ',' << total_T.y() << ',' << total_T.z() << ','
                 << gimbal_T.x() << ',' << gimbal_T.y() << ',' << gimbal_T.z() << ','
                 << roll_T << ',' << pitch_T << ',' << yaw_T << ','
                 << roll_I << ',' << pitch_I << ',' << yaw_I << '\n';
            csv_.flush();
        }
    }

    // ---------------------------  math helpers  ----------------------------- //
    static void extract_euler(const Eigen::Matrix3d& R, double& phi, double& theta, double& psi) {
        theta = -std::asin(R(0,2));
        theta = std::clamp(theta, -M_PI/2.0, M_PI/2.0);
        if (std::abs(std::cos(theta)) > 1e-6) {
            phi = std::atan2(R(1,2), R(2,2));
            psi = std::atan2(R(0,1), R(0,0));
        } else {
            phi = 0.0;
            psi = (theta > 0 ? 1 : -1) * std::atan2(-R(1,0), -R(1,1));
        }
    }

    static void euler_rates(double phi, double theta, double p, double q, double r,
                            double& phi_dot, double& theta_dot, double& psi_dot) {
        const double cphi = std::cos(phi), sphi = std::sin(phi), cth = std::cos(theta);
        if (std::abs(cth) < 1e-6) {
            phi_dot = p;
            theta_dot = q*cphi - r*sphi;
            psi_dot = 0.0;
        } else {
            const double tan_th = std::tan(theta);
            phi_dot   = p + q*sphi*tan_th + r*cphi*tan_th;
            theta_dot =     q*cphi        - r*sphi;
            psi_dot   =     q*sphi/cth    + r*cphi/cth;
        }
    }

    void update_dcms(double phi, double theta) {
        const double cphi = std::cos(phi), sphi = std::sin(phi);
        const double cth  = std::cos(theta), sth = std::sin(theta);
        C_J_LPR_ << 1,0,0, 0,cphi,sphi, 0,-sphi,cphi;
        C_J_LYR_ << cth, sphi*sth, cphi*sth,
                    0,   cphi,     -sphi,
                    -sth,sphi*cth, cphi*cth;
    }

    static Eigen::Matrix3d inv_jacobian(double phi, double theta) {
        const double cphi = std::cos(phi), sphi = std::sin(phi), cth = std::cos(theta);
        Eigen::Matrix3d Ginv = Eigen::Matrix3d::Identity();
        if (std::abs(cth) < 1e-6) return Ginv;   // gimbal lock safeguard
        const double tan_th = std::tan(theta);
        Ginv << 1, sphi*tan_th, cphi*tan_th,
                 0, cphi,      -sphi,
                 0, sphi/cth,   cphi/cth;
        return Ginv;
    }

    static int32_t torque_to_current(double T_nm) {
        const double current_A = (T_nm >= 0.0) ? 0.6370*T_nm + 0.0395
                                               : 0.6439*T_nm - 0.1097;
        double raw = current_A/CURRENT_LSB + CURRENT_OFFSET_RAW;
        raw = std::clamp(raw, static_cast<double>(CURRENT_RAW_MIN), static_cast<double>(CURRENT_RAW_MAX));
        return static_cast<int32_t>(std::lround(raw));
    }

    void write_byte(uint8_t id, uint16_t addr, uint8_t val, const std::string& what) {
        uint8_t dxl_err = 0;
        int rc = packet_->write1ByteTxRx(port_, id, addr, val, &dxl_err);
        if (rc != COMM_SUCCESS)
            RCLCPP_ERROR(get_logger(), "DXL %d %s comm error: %s", id, what.c_str(), packet_->getTxRxResult(rc));
        else if (dxl_err)
            RCLCPP_ERROR(get_logger(), "DXL %d %s proto error: %s", id, what.c_str(), packet_->getRxPacketError(dxl_err));
    }

    void send_currents(int32_t roll_I, int32_t pitch_I, int32_t yaw_I) {
        if (!group_write_) return;
        uint8_t buf[LEN_GOAL_CURRENT];
        auto add_param = [&](uint8_t id, int32_t cur){
            const uint16_t w = static_cast<uint16_t>(cur);
            buf[0] = DXL_LOBYTE(w);
            buf[1] = DXL_HIBYTE(w);
            return group_write_->addParam(id, buf);
        };
        bool ok = add_param(DXL_ID_ROLL, roll_I) && add_param(DXL_ID_PITCH, pitch_I) && add_param(DXL_ID_YAW, yaw_I);
        if (!ok) { group_write_->clearParam(); return; }
        int rc = group_write_->txPacket();
        if (rc != COMM_SUCCESS)
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "SyncWrite error: %s", packet_->getTxRxResult(rc));
        group_write_->clearParam();
    }

    void disable_torque_all() {
        for (uint8_t id : DXL_IDS) write_byte(id, ADDR_TORQUE_ENABLE, 0, "torque disable");
    }

    // ------------------------------  members  ------------------------------ //
    // ROS
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // state
    std::mutex mu_;
    px4_msgs::msg::VehicleOdometry::SharedPtr odom_;
    bool  have_odom_ = false;
    bool  first_     = true;
    rclcpp::Time prev_time_;

    // dyn data
    double phi_{0.0}, theta_{0.0}, psi_{0.0};
    double p_raw_{0.0}, q_raw_{0.0}, r_raw_{0.0};
    double p_dot_{0.0}, q_dot_{0.0}, r_dot_{0.0};
    double phi_dot_{0.0}, theta_dot_{0.0}, psi_dot_{0.0};

    // filters
    std::unique_ptr<FilterDiff> fd_p_, fd_q_, fd_r_;

    // kinematics
    Eigen::Matrix3d C_J_I_;
    Eigen::Matrix3d C_J_LPR_, C_J_LYR_;

    // inertia
    const Eigen::Matrix3d I_CUAV_, I_CRB_, I_CPR_, I_CYR_;

    // dynamics helper
    EqDynamics dyn_;

    // DXL SDK
    dynamixel::PortHandler* port_ = nullptr;
    dynamixel::PacketHandler* packet_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncWrite> group_write_;

    // csv
    std::ofstream csv_;
    std::string   csv_name_;
};

// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DroneDxlBridge>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
