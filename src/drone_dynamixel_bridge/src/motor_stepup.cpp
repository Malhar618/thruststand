#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class MotorTestStepUp : public rclcpp::Node
{
public:
  MotorTestStepUp() : Node("motor_test_stepup")
  {
    // Start at 5% power, step up by 5% until 100%
    current_power_ = 0.05f;
    max_power_ = 1.0f;
    increment_ = 0.05f;
    // ActuatorMotors message typically has a fixed array of 12 floats
    num_channels_ = 12;

    publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);

    // Publish actuator command at 10 Hz
    publish_timer_ = this->create_wall_timer(100ms, std::bind(&MotorTestStepUp::publish_command, this));
    // Update motor power every 15 seconds
    step_timer_ = this->create_wall_timer(15s, std::bind(&MotorTestStepUp::step_power, this));

    RCLCPP_INFO(this->get_logger(), "Step-Up Test Started: Power = %.2f%%", current_power_ * 100);
  }

private:
  void publish_command()
  {
    auto msg = px4_msgs::msg::ActuatorMotors();
    // Since msg.control is a fixed-size std::array, fill it with zeros
    std::fill(msg.control.begin(), msg.control.end(), 0.0f);
    // Command only motor 1 (channel 0)
    msg.control[1] = current_power_;
    publisher_->publish(msg);
  }

  void step_power()
  {
    if (current_power_ < max_power_) {
      current_power_ += increment_;
      if (current_power_ > max_power_)
        current_power_ = max_power_;
      RCLCPP_INFO(this->get_logger(), "Stepped up power to %.2f%%", current_power_ * 100);
    } else {
      RCLCPP_INFO(this->get_logger(), "Maximum power reached: %.2f%%", current_power_ * 100);
    }
  }

  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  float current_power_;
  float max_power_;
  float increment_;
  size_t num_channels_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorTestStepUp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
