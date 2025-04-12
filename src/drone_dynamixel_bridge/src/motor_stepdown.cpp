#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include <chrono>
#include <algorithm> // For std::max, std::min
#include <cmath>     // For NAN
#include <thread>    // Required for std::this_thread::sleep_for (though not used directly in final version, good practice)

using namespace std::chrono_literals;

class MotorStepDownTest : public rclcpp::Node
{
public:
  MotorStepDownTest() : Node("motor_stepdown_test"), armed_(false)
  {
    // Initialize thrust: start at 100% (1.0) and step down by 5% until 0% (0.0)
    current_power_ = 1.0f;
    min_power_ = 0.0f; // Target minimum power
    increment_ = 0.05f; // Step size for decrease
    // Use 12 channels (as defined in the actuator_motors message)
    num_channels_ = 12; // Although we only use one motor channel

    // Publishers:
    actuator_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    // Timer to publish offboard control and actuator commands at 10 Hz
    // IMPORTANT: This timer starts immediately but waits for arming before sending actual motor commands.
    publish_timer_ = this->create_wall_timer(100ms, std::bind(&MotorStepDownTest::publish_offboard_commands, this));

    // Timer to update thrust every 15 seconds
    // This timer will be started *after* arming
    step_timer_ = this->create_wall_timer(15s, std::bind(&MotorStepDownTest::step_power, this));
    // Initially disable the step timer until arming is attempted
    step_timer_->cancel();

    // Timer to attempt arming after a short delay (e.g., 1 second)
    arm_timer_ = this->create_wall_timer(1s, std::bind(&MotorStepDownTest::send_arm_command, this));

    RCLCPP_INFO(this->get_logger(), "Motor Step-Down Test Node Started. Initial Thrust = %.2f%%", current_power_ * 100);
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

    // Give PX4 a moment to process the arm command before starting thrust decrease
    // Start the step timer only after attempting to arm
    RCLCPP_INFO(this->get_logger(), "Starting power step-down sequence.");
    step_timer_->reset(); // Start the 15s countdown for the first step-down
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
      motor_msg.control[0] = std::max(0.0f, std::min(1.0f, current_power_));

      actuator_pub_->publish(motor_msg);
    }
  }

  // Decrease thrust by 5% every 15 seconds until 0% is reached
  void step_power()
  {
    // Only step power if armed
    if (!armed_) {
        RCLCPP_WARN(this->get_logger(), "Cannot step power, vehicle not armed.");
        return;
    }

    if (current_power_ > min_power_) {
      current_power_ -= increment_;
      // Clamp the value just in case it goes slightly below min_power due to floating point inaccuracies
      current_power_ = std::max(current_power_, min_power_);
      RCLCPP_INFO(this->get_logger(), "Stepped down thrust to %.2f%%", current_power_ * 100);
    }

    // Check if minimum power has been reached or passed
    if (current_power_ <= min_power_) {
        RCLCPP_INFO(this->get_logger(), "Minimum thrust reached: %.2f%%. Test finished.", min_power_ * 100);
        // Optionally stop timers or disarm here
        // publish_timer_->cancel(); // Stop sending commands
        step_timer_->cancel();      // Stop trying to step down further
        // send_disarm_command(); // Implement and call a disarm function if desired
        // rclcpp::shutdown(); // Or shutdown the node entirely
    }
  }

  // Optional: Add a disarm function if needed
  // void send_disarm_command() { ... }

  // Publishers and timers
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr arm_timer_;

  // Thrust variables
  float current_power_; // Starts high, decreases
  float min_power_;     // Target minimum
  float increment_;     // Amount to decrease by
  size_t num_channels_;

  // State flag
  bool armed_; // Use this to control when to send motor commands
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorStepDownTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
