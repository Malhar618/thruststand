#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include <chrono>
#include <algorithm>
#include <thread> // Required for std::this_thread::sleep_for

using namespace std::chrono_literals;

class MotorStepUpTest : public rclcpp::Node
{
public:
  MotorStepUpTest() : Node("motor_stepup_test"), armed_(false)
  {
    // Initialize thrust: start at 5% (0.05) and step up by 5% until 100% (1.0)
    current_power_ = 0.05f;
    max_power_ = 1.0f;
    increment_ = 0.05f;
    // Use 12 channels (as defined in the actuator_motors message)
    num_channels_ = 12; // Although we only use one motor channel

    // Publishers:
    actuator_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    // Timer to publish offboard control and actuator commands at 10 Hz
    // IMPORTANT: This timer starts immediately but waits for arming before sending actual motor commands.
    publish_timer_ = this->create_wall_timer(100ms, std::bind(&MotorStepUpTest::publish_offboard_commands, this));

    // Timer to update thrust every 15 seconds
    step_timer_ = this->create_wall_timer(15s, std::bind(&MotorStepUpTest::step_power, this));
    // Timer to attempt arming after a short delay (e.g., 1 second)
    // We might need to send arm command repeatedly until confirmed, but start with one attempt.
    arm_timer_ = this->create_wall_timer(1s, std::bind(&MotorStepUpTest::send_arm_command, this));

    RCLCPP_INFO(this->get_logger(), "Motor Step-Up Test Node Started. Initial Thrust = %.2f%%", current_power_ * 100);
    RCLCPP_INFO(this->get_logger(), "Attempting to arm and enter offboard mode shortly...");
    RCLCPP_WARN(this->get_logger(), "Ensure propellers are REMOVED for safety!");

  }

private:
  // Send arm command
  void send_arm_command()
  {
    // Cancel timer to only send arm command once initially
    // In a real application, you might monitor vehicle status and resend if arming fails.
    arm_timer_->cancel();

    // Publish vehicle command to arm the vehicle (command 400)
    px4_msgs::msg::VehicleCommand arm_cmd{};
    arm_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_cmd.param1 = 1.0f; // 1.0 to arm
    arm_cmd.target_system = 1;
    arm_cmd.target_component = 1;
    arm_cmd.source_system = 1; // Should match MAV_SYSTEM_ID of your ROS 2 companion
    arm_cmd.source_component = 1; // Should match MAV_COMPONENT_ID of your ROS 2 companion
    arm_cmd.from_external = true;
    vehicle_cmd_pub_->publish(arm_cmd);
    RCLCPP_INFO(this->get_logger(), "Arm command sent.");

    // Set the armed flag optimistically. A robust solution would subscribe to VehicleStatus
    // to confirm arming state.
    armed_ = true;

    // Give PX4 a moment to process the arm command before starting thrust increase
    // Start the step timer only after attempting to arm
    RCLCPP_INFO(this->get_logger(), "Starting power step-up sequence.");
    step_timer_->reset(); // Start the 15s countdown for the first step
  }

  // Publish OffboardControlMode and ActuatorMotors commands continuously
  void publish_offboard_commands()
  {
    // Always publish OffboardControlMode to keep the mode active
    px4_msgs::msg::OffboardControlMode offb_msg{};
    offb_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offb_msg.position = false;
    offb_msg.velocity = false;
    offb_msg.acceleration = false;
    offb_msg.attitude = false;
    offb_msg.body_rate = false;
    offb_msg.thrust_and_torque = false; // Set to false if using ActuatorMotors
    offb_msg.direct_actuator = true;    // Enable direct actuator control
    offboard_mode_pub_->publish(offb_msg);

    // Only publish actuator commands if armed
    if (armed_) {
      auto motor_msg = px4_msgs::msg::ActuatorMotors();
      motor_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      // Initialize all controls to NaN (Not a Number) or a safe value
      // PX4 ignores NaN control values. Set unused motors explicitly to NaN.
      std::fill(motor_msg.control.begin(), motor_msg.control.end(), NAN);

      // Command only motor 1 (index 0).
      // Ensure the value is within the expected range [0, 1] for normalized thrust.
      // PX4 might also map [-1, 1] depending on configuration, but 0-1 is typical for direct motor control.
      motor_msg.control[0] = std::max(0.0f, std::min(1.0f, current_power_));

      actuator_pub_->publish(motor_msg);
    }
  }

  // Increase thrust by 5% every 15 seconds until 100% is reached
  void step_power()
  {
    // Only step power if armed
    if (!armed_) {
        RCLCPP_WARN(this->get_logger(), "Cannot step power, vehicle not armed.");
        return;
    }

    if (current_power_ < max_power_) {
      current_power_ += increment_;
      // Clamp the value just in case
      current_power_ = std::min(current_power_, max_power_);
      RCLCPP_INFO(this->get_logger(), "Stepped thrust to %.2f%%", current_power_ * 100);
    }

    if (current_power_ >= max_power_) {
        RCLCPP_INFO(this->get_logger(), "Maximum thrust reached: %.2f%%. Test finished.", max_power_ * 100);
        // Optionally stop timers or disarm here
        // publish_timer_->cancel();
        // step_timer_->cancel();
        // send_disarm_command(); // Implement a disarm function
        // rclcpp::shutdown(); // Or shutdown the node
    }
  }

  // Publishers and timers
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr arm_timer_;

  // Thrust variables
  float current_power_;
  float max_power_;
  float increment_;
  size_t num_channels_;

  // State flag
  bool armed_; // Use this to control when to send motor commands
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorStepUpTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
