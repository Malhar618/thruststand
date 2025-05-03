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

const char*   DEVICENAME     = "/dev/ttyUSB1";
const int     BAUDRATE       = 4'000'000;
const float   PROTOCOL_VER   = 2.0f;

const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_GOAL_CURRENT  = 102;
const uint16_t LEN_TORQUE_ENABLE  = 1;
const uint16_t LEN_GOAL_CURRENT   = 2;

const double   CURRENT_LSB        = 0.0045;   // 4.5 mA per tick
const int      CURRENT_OFFSET_RAW = 2048;     // register value @ 0 A

// -----------------------------------------------------------------------------
//                    Patch‑C constants (damping & clamp)
// -----------------------------------------------------------------------------
const double  KD_ROLL   = 1.5;
const double  KD_PITCH  = 1.0;
const double  KD_YAW    = 1.0;
const int16_t RAW_LIMIT = 50;   // ±1200 ticks ≈ 3 A

// -----------------------------------------------------------------------------
//                               Helper classes
// -----------------------------------------------------------------------------
class FilterDiff {
public:
    FilterDiff(double wn, double zeta)
        : wn_(wn), zeta_(zeta) {}
    void update(double in, double dt){
        if(dt<=1e-9) return;
        if(!init_){ x1_=in; x2_=0.0; init_=true; }
        double x1d=x2_;
        double x2d=-wn_*wn_*x1_ - 2.0*zeta_*wn_*x2_ + wn_*wn_*in;
        x1_+=x1d*dt; x2_+=x2d*dt;
    }
    double derivative() const { return x2_; }
    void reset(double v0=0.0){ x1_=v0; x2_=0.0; init_=true; }
private:
    double wn_,zeta_,x1_{0.0},x2_{0.0}; bool init_{false};
};

// Patch‑D LPF
class LP1 {
    double y_{0.0}, tau_; bool init_{false};
public:
    explicit LP1(double fc_hz):tau_(1.0/(2*M_PI*fc_hz)){}
    double filter(double x,double dt){
        if(!init_){ y_=x; init_=true; return y_; }
        y_ += (dt/(tau_+dt))*(x-y_);
        return y_;
    }
};

class EqDynamics {
public:
    Eigen::Vector3d compute_torque(const Eigen::Matrix3d& I,
                                   const Eigen::Vector3d& w,
                                   const Eigen::Vector3d& wdot) const {
        return I*wdot + w.cross(I*w);
    }
};

// -----------------------------------------------------------------------------
//                               Main ROS 2 Node
// -----------------------------------------------------------------------------
class DroneDxlBridge : public rclcpp::Node {
public:
    DroneDxlBridge()
        : Node("drone_dynamixel_bridge_node"),
          I_CUAV_((Eigen::Matrix3d() << 0.012576,-0.000001,-0.000069,
                                       -0.000001, 0.019215, 0.000012,
                                       -0.000069, 0.000012, 0.026610).finished()),
          I_CRB_ ((Eigen::Matrix3d() << 0.000866,-0.000001,-0.000068,
                                       -0.000001, 0.101389, 0.000004,
                                       -0.000068, 0.000004, 0.101225).finished()),
          I_CPR_ ((Eigen::Matrix3d() << 0.640302, 0.000043, 0.000078,
                                        0.000043, 0.570491,-0.000025,
                                        0.000078,-0.000025, 1.200050).finished()),
          I_CYR_ ((Eigen::Matrix3d() << 2.384163,-0.000049,-0.000104,
                                       -0.000049, 1.110062, 0.000781,
                                       -0.000104, 0.000781, 1.286313).finished())
    {
        RCLCPP_INFO(get_logger(),"Init …");
        init_csv();  init_filters();  init_sdk();  init_ros();
        RCLCPP_INFO(get_logger(),"Node ready.");
    }
    ~DroneDxlBridge() override {
        disable_torque_all();
        if(port_) port_->closePort();
        if(csv_.is_open()) csv_.close();
    }

private:
    // -----------------------  Initialisation helpers  ----------------------- //
    void init_csv(){
        auto now=std::chrono::system_clock::now();
        std::time_t t=std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss; oss<<"diag_"<<std::put_time(std::localtime(&t),"%Y%m%d_%H%M%S")<<".csv";
        csv_name_=oss.str();
        csv_.open(csv_name_,std::ios::out);
        if(csv_.is_open()){
            csv_<<"stamp,phi,theta,psi,p,q,r,p_dot,q_dot,r_dot,"
                   "Tix,Tiy,Tiz,Gx,Gy,Gz,cmdTroll,cmdTpitch,cmdTyaw,"
                   "rawIroll,rawIpitch,rawIyaw\n";
        }
    }
    void init_filters(){ fd_p_=std::make_unique<FilterDiff>(30.0,0.7);
                         fd_q_=std::make_unique<FilterDiff>(30.0,0.7);
                         fd_r_=std::make_unique<FilterDiff>(30.0,0.7); }
    void init_sdk(){
        port_=dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_=dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VER);
        if(!port_->openPort()||!port_->setBaudRate(BAUDRATE))
            throw std::runtime_error("DXL port error");
        for(uint8_t id:DXL_IDS) write_byte(id,ADDR_TORQUE_ENABLE,1,"torque enable");
        group_write_=std::make_unique<dynamixel::GroupSyncWrite>(port_,packet_,ADDR_GOAL_CURRENT,LEN_GOAL_CURRENT);
    }
    void init_ros(){
        using px4_msgs::msg::VehicleOdometry;
        odom_sub_=create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry",rclcpp::QoS(10).best_effort(),
            [&](VehicleOdometry::SharedPtr m){ std::lock_guard<std::mutex> lk(mu_); odom_=std::move(m); have_odom_=true; });
        timer_=create_wall_timer(5ms,[&]{ timer_cb(); });
        prev_time_=now();
    }

    // ------------------------------  Core loop  ----------------------------- //
    void timer_cb(){
        rclcpp::Time t_now=now();
        double dt=(t_now-prev_time_).seconds();
        if(dt<=1e-6||dt>0.1) return;
        prev_time_=t_now;

        px4_msgs::msg::VehicleOdometry::SharedPtr od;
        { std::lock_guard<std::mutex> lk(mu_);
          if(!have_odom_) return;
          od=odom_; }

        // body rates
        p_raw_=od->angular_velocity[0];
        q_raw_=od->angular_velocity[1];
        r_raw_=od->angular_velocity[2];

        // attitude
        Eigen::Quaterniond q_I_J(od->q[0],od->q[1],od->q[2],od->q[3]); q_I_J.normalize();
        C_J_I_=q_I_J.toRotationMatrix().transpose();
        extract_euler(C_J_I_,phi_,theta_,psi_);

        // filtered derivatives
        if(first_){ fd_p_->reset(p_raw_); fd_q_->reset(q_raw_); fd_r_->reset(r_raw_); first_=false; return; }
        fd_p_->update(p_raw_,dt); fd_q_->update(q_raw_,dt); fd_r_->update(r_raw_,dt);
        p_dot_=fd_p_->derivative(); q_dot_=fd_q_->derivative(); r_dot_=fd_r_->derivative();

        // kinematics helpers
        euler_rates(phi_,theta_,p_raw_,q_raw_,r_raw_,phi_dot_,theta_dot_,psi_dot_);
        update_dcms(phi_,theta_);

        // ---------------- Inertia aggregation (Patch B) ----------------
        auto rotJ=[&](const Eigen::Matrix3d& C,const Eigen::Matrix3d& I){ return C*I*C.transpose(); };
        Eigen::Matrix3d I_tot = I_CRB_ + rotJ(C_J_LPR_,I_CPR_) + rotJ(C_J_LYR_,I_CYR_);
        Eigen::Vector3d wJ(p_raw_,q_raw_,r_raw_), wdotJ(p_dot_,q_dot_,r_dot_);
        Eigen::Vector3d total_T = I_tot*wdotJ + wJ.cross(I_tot*wJ);

        // LPF 20 Hz (Patch D)
        total_T.x()=lpf_tx_.filter(total_T.x(),dt);
        total_T.y()=lpf_ty_.filter(total_T.y(),dt);
        total_T.z()=lpf_tz_.filter(total_T.z(),dt);

        // -------------- Γ⁻¹ mapping (Patch A) --------------------------
        double cth=std::cos(theta_);
        if(std::abs(cth)<1e-4) return;         // near singular
        double cpsi=std::cos(psi_), spsi=std::sin(psi_);
        double Tx=total_T.x(), Ty=total_T.y(), Tz=total_T.z();
        double roll_T  =(Tx*cpsi + Ty*spsi)/cth;
        double pitch_T = Ty*cpsi - Tx*spsi;
        double yaw_T   = Tz - std::tan(theta_)*(Tx*cpsi + Ty*spsi);

        // viscous rate damping (Patch C)
        roll_T  -= KD_ROLL  * p_raw_;
        pitch_T -= KD_PITCH * q_raw_;
        yaw_T   -= KD_YAW   * r_raw_;

        // create gimbal_T for logging
        Eigen::Vector3d gimbal_T(roll_T,pitch_T,yaw_T);

        //------------------ current commands ---------------------------
        int32_t roll_I = torque_to_current(roll_T);
        int32_t pitch_I= torque_to_current(pitch_T);
        int32_t yaw_I  = torque_to_current(yaw_T);
        send_currents(roll_I,pitch_I,yaw_I);

        // ------------------------- log -------------------------------
        if(csv_.is_open()){
            csv_<<std::fixed<<std::setprecision(6)
                <<t_now.seconds()<<','<<phi_<<','<<theta_<<','<<psi_<<','
                <<p_raw_<<','<<q_raw_<<','<<r_raw_<<','
                <<p_dot_<<','<<q_dot_<<','<<r_dot_<<','
                <<total_T.x()<<','<<total_T.y()<<','<<total_T.z()<<','
                <<gimbal_T.x()<<','<<gimbal_T.y()<<','<<gimbal_T.z()<<','
                <<roll_T<<','<<pitch_T<<','<<yaw_T<<','
                <<roll_I<<','<<pitch_I<<','<<yaw_I<<'\n';
        }
    }

    // --------------------------- math helpers ------------------------------ //
    static void extract_euler(const Eigen::Matrix3d& R,double& phi,double& theta,double& psi){
        theta=-std::asin(R(0,2));
        theta=std::clamp(theta,-M_PI/2.0,M_PI/2.0);
        if(std::abs(std::cos(theta))>1e-6){ phi=std::atan2(R(1,2),R(2,2)); psi=std::atan2(R(0,1),R(0,0)); }
        else{ phi=0.0; psi=(theta>0?1:-1)*std::atan2(-R(1,0),-R(1,1)); }
    }
    static void euler_rates(double phi,double theta,double p,double q,double r,
                            double& pd,double& td,double& psid){
        double cphi=std::cos(phi), sphi=std::sin(phi), cth=std::cos(theta);
        if(std::abs(cth)<1e-6){ pd=p; td=q*cphi - r*sphi; psid=0.0; }
        else{
            double tan_th=std::tan(theta);
            pd = p + q*sphi*tan_th + r*cphi*tan_th;
            td = q*cphi - r*sphi;
            psid= q*sphi/cth + r*cphi/cth;
        }
    }
    void update_dcms(double phi,double theta){
        double cphi=std::cos(phi), sphi=std::sin(phi);
        double cth=std::cos(theta), sth=std::sin(theta);
        C_J_LPR_<<1,0,0, 0,cphi,sphi, 0,-sphi,cphi;
        C_J_LYR_<< cth, sphi*sth, cphi*sth,
                     0,   cphi,     -sphi,
                   -sth, sphi*cth, cphi*cth;
    }
    static int32_t torque_to_current(double T_nm){
        double I_A = (T_nm>=0.0)? 0.6370*T_nm + 0.0395
                                 : 0.6439*T_nm - 0.1097;
        double raw = I_A/CURRENT_LSB + CURRENT_OFFSET_RAW;
        raw=std::clamp(raw,
                       static_cast<double>(CURRENT_OFFSET_RAW-RAW_LIMIT),
                       static_cast<double>(CURRENT_OFFSET_RAW+RAW_LIMIT));
        return static_cast<int32_t>(std::lround(raw));
    }

    // --------------------------- DXL helpers ------------------------------- //
    void write_byte(uint8_t id,uint16_t addr,uint8_t val,const char* what){
        uint8_t dxl_err=0;
        int rc=packet_->write1ByteTxRx(port_,id,addr,val,&dxl_err);
        if(rc!=COMM_SUCCESS)
            RCLCPP_ERROR(get_logger(),"DXL %d %s comm err",id,what);
        else if(dxl_err)
            RCLCPP_ERROR(get_logger(),"DXL %d %s proto err",id,what);
    }
    void send_currents(int32_t r,int32_t p,int32_t y){
        if(!group_write_) return;
        uint8_t buf[LEN_GOAL_CURRENT];
        auto add=[&](uint8_t id,int32_t cur){
            uint16_t w=static_cast<uint16_t>(cur);
            buf[0]=DXL_LOBYTE(w); buf[1]=DXL_HIBYTE(w);
            return group_write_->addParam(id,buf);
        };
        if(!(add(DXL_ID_ROLL,r)&&add(DXL_ID_PITCH,p)&&add(DXL_ID_YAW,y))){
            group_write_->clearParam(); return;
        }
        int rc=group_write_->txPacket();
        if(rc!=COMM_SUCCESS)
            RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"SyncWrite err");
        group_write_->clearParam();
    }
    void disable_torque_all(){ for(uint8_t id:DXL_IDS) write_byte(id,ADDR_TORQUE_ENABLE,0,"disable"); }

    // ------------------------------  members  ------------------------------ //
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mu_;
    px4_msgs::msg::VehicleOdometry::SharedPtr odom_;
    bool have_odom_{false};
    bool first_{true};
    rclcpp::Time prev_time_;

    double phi_{},theta_{},psi_{};
    double p_raw_{},q_raw_{},r_raw_{};
    double p_dot_{},q_dot_{},r_dot_{};
    double phi_dot_{},theta_dot_{},psi_dot_{};

    std::unique_ptr<FilterDiff> fd_p_,fd_q_,fd_r_;
    LP1 lpf_tx_{10.0}, lpf_ty_{10.0}, lpf_tz_{10.0};

    Eigen::Matrix3d C_J_I_,C_J_LPR_,C_J_LYR_;
    const Eigen::Matrix3d I_CUAV_,I_CRB_,I_CPR_,I_CYR_;
    EqDynamics dyn_;

    dynamixel::PortHandler* port_{nullptr};
    dynamixel::PacketHandler* packet_{nullptr};
    std::unique_ptr<dynamixel::GroupSyncWrite> group_write_;

    std::ofstream csv_; std::string csv_name_;
};

// ---------------------------------------------------------------------------
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    try{ rclcpp::spin(std::make_shared<DroneDxlBridge>()); }
    catch(const std::exception& e){
        RCLCPP_FATAL(rclcpp::get_logger("main"),"fatal: %s",e.what());
    }
    rclcpp::shutdown();
    return 0;
}
