#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msgs/vehicle_odometry.hpp>
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <mutex>
#include <algorithm>  // for std::max, std::min

// === Adjust these for your hardware === //
static constexpr uint8_t  DXL_ID              = 3;         // Dynamixel servo ID
static constexpr char      DEVICENAME[]        = "/dev/ttyUSB1"; // USB port for the servo
static constexpr int       BAUDRATE            = 57600;     // or whatever your servo uses
static constexpr uint16_t  ADDR_OPERATING_MODE = 11;
static constexpr uint16_t  ADDR_TORQUE_ENABLE  = 64;
static constexpr uint16_t  ADDR_GOAL_POSITION  = 116;
// Position control mode for many Dynamixels
static constexpr uint8_t   POSITION_CONTROL_MODE = 3;

// 0..4095 = 0..360°, so ~11.377 ticks per degree
static constexpr double TICKS_PER_DEGREE = 4096.0 / 360.0;
class DroneDynamixelBridge : public rclcpp::Node
{
public:
    DroneDynamixelBridge()
    : Node("drone_dynamixel_bridge")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing DroneDynamixelBridge node...");

        // 1) Subscribe to /fmu/out/vehicle_attitude using best-effort QoS
        auto qos_profile = rclcpp::QoS(10).best_effort();
        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos_profile,
            std::bind(&DroneDynamixelBridge::attitudeCallback, this, std::placeholders::_1)
        );

        // 2) Initialize the Dynamixel port and packet handlers
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

        // 3) Set position control mode and enable torque
        write1Byte(DXL_ID, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE);
        write1Byte(DXL_ID, ADDR_TORQUE_ENABLE, 1);

        RCLCPP_INFO(this->get_logger(), "Dynamixel is ready to mirror drone roll angles.");
    }

    ~DroneDynamixelBridge()
    {
        // Disable torque on shutdown
        write1Byte(DXL_ID, ADDR_TORQUE_ENABLE, 0);
        portHandler_->closePort();
    }

private:
    // Callback for the /fmu/out/vehicle_attitude topic
    void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // PX4 quaternion = [w, x, y, z] = [q0, q1, q2, q3]
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];

        // Convert to roll/pitch/yaw using TF2
        // Reorder to (x, y, z, w) because PX4's array is [w, x, y, z]
        tf2::Quaternion tf2_quat(q1, q2, q3, q0);
        tf2::Matrix3x3 tf2_matrix(tf2_quat);

        double roll, pitch, yaw;
        tf2_matrix.getRPY(roll, pitch, yaw);

        // Convert roll to degrees
        double roll_deg = roll * 180.0 / M_PI;

        // (Optional) clamp the roll angle (e.g., ±45° to avoid servo damage)
        roll_deg = std::max(-45.0, std::min(45.0, roll_deg));

        // Convert degrees to servo position ticks.
        //  - 2048 = "neutral" position
        //  - shift by roll_deg * TICKS_PER_DEGREE
        double servo_pos = 2048.0 + (roll_deg * TICKS_PER_DEGREE);

        // Bound the position to [0, 4095].
        if (servo_pos < 0) {
            servo_pos = 0;
        } else if (servo_pos > 4095) {
            servo_pos = 4095;
        }

        // Write goal position to the Dynamixel
        write4Byte(DXL_ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(servo_pos));

        // Log occasionally (once every 2 seconds)
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000, // milliseconds
            "Drone roll=%.1f deg => servo=%.1f ticks",
            roll_deg, servo_pos
        );
    }

    // Helper to write 1 byte
    void write1Byte(uint8_t dxl_id, uint16_t addr, uint8_t value)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(
            portHandler_, dxl_id, addr, value, &dxl_error
        );
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Comm fail write1Byte: %s",
                         packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error) {
            RCLCPP_ERROR(this->get_logger(), "DXL error write1Byte: %s",
                         packetHandler_->getRxPacketError(dxl_error));
        }
    }

    // Helper to write 4 bytes
    void write4Byte(uint8_t dxl_id, uint16_t addr, uint32_t value)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write4ByteTxRx(
            portHandler_, dxl_id, addr, value, &dxl_error
        );
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Comm fail write4Byte: %s",
                         packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error) {
            RCLCPP_ERROR(this->get_logger(), "DXL error write4Byte: %s",
                         packetHandler_->getRxPacketError(dxl_error));
        }
    }

    // ROS subscription for the drone’s attitude
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;

    // Dynamixel variables
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
};

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create the node
    auto node = std::make_shared<DroneDynamixelBridge>();
    // Spin until shutdown
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
