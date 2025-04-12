#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleAttitudeSetpoint;

class PerformanceIndexNode : public rclcpp::Node
{
public:
  PerformanceIndexNode()
  : Node("performance_index_node"),
    accumulated_error_sq_(0.0),
    mission_duration_sec_(30.0), // Set mission duration here based on what Dr L'Afflitto Recoommends. 
    actual_received_(false),
    desired_received_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing PerformanceIndexNode...");

    // Subscribe to actual vehicle attitude from PX4 (topic: /fmu/out/vehicle_attitude)
    auto qos_actual = rclcpp::QoS(10).best_effort();
    actual_sub_ = this->create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_actual,
      std::bind(&PerformanceIndexNode::actualAttitudeCallback, this, std::placeholders::_1)
    );

    // Subscribe to desired vehicle attitude setpoint (topic: /fmu/in/vehicle_attitude_setpoint)
    auto qos_desired = rclcpp::QoS(10).best_effort();
    desired_sub_ = this->create_subscription<VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", qos_desired,
      std::bind(&PerformanceIndexNode::desiredAttitudeCallback, this, std::placeholders::_1)
    );

    // Timer callback runs at 10 Hz (every 100 ms)
    timer_ = this->create_wall_timer(100ms, std::bind(&PerformanceIndexNode::timerCallback, this));
    prev_time_ = this->now();
  }

private:
  // Subscribers and Timer
  rclcpp::Subscription<VehicleAttitude>::SharedPtr actual_sub_;
  rclcpp::Subscription<VehicleAttitudeSetpoint>::SharedPtr desired_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Mutex for thread safety
  std::mutex mutex_;

  // Latest received Euler angles (roll, pitch, yaw in radians)
  Eigen::Vector3d actual_attitude_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_attitude_{Eigen::Vector3d::Zero()};
  bool actual_received_;
  bool desired_received_;

  // Variables for performance index calculation
  double accumulated_error_sq_;
  rclcpp::Time prev_time_;
  rclcpp::Time mission_start_time_;
  bool mission_started_ = false;
  double mission_duration_sec_;

  // Callback for actual vehicle attitude
  void actualAttitudeCallback(const VehicleAttitude::SharedPtr msg)
  {
    // Convert quaternion (PX4 uses [w, x, y, z]) to Euler angles (roll, pitch, yaw)
    tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      actual_attitude_ = Eigen::Vector3d(roll, pitch, yaw);
      actual_received_ = true;
      if (!mission_started_) {
        mission_start_time_ = this->now();
        mission_started_ = true;
      }
    }
  }

  // Callback for desired attitude setpoint
  void desiredAttitudeCallback(const VehicleAttitudeSetpoint::SharedPtr msg)
  {
    // Assume desired Euler angles are provided in fields: roll_body, pitch_body, yaw_body (in radians)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      desired_attitude_ = Eigen::Vector3d(msg->roll_body, msg->pitch_body, msg->yaw_body);
      desired_received_ = true;
    }
  }

  // Timer callback: integrates squared error over time and computes performance index at mission end
  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    Eigen::Vector3d actual, desired;
    bool compute_error = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (actual_received_ && desired_received_) {
        actual = actual_attitude_;
        desired = desired_attitude_;
        compute_error = true;
      }
    }

    if (compute_error) {
      // Compute error vector in Euler angles
      Eigen::Vector3d error = desired - actual;
      accumulated_error_sq_ += error.squaredNorm() * dt;
      double elapsed = (now - mission_start_time_).seconds();

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Elapsed: %.2fs, Instant L2 Error: %.4f", elapsed, std::sqrt(error.squaredNorm()));

      // Once mission duration is reached, compute and write performance index
      if (elapsed >= mission_duration_sec_) {
        double performance_index = std::sqrt(accumulated_error_sq_);
        RCLCPP_INFO(this->get_logger(),
          "Mission complete. Performance Index (L2 norm integrated over time): %.4f", performance_index);

        // Write performance index to file
        std::ofstream outfile("performance_index.txt");
        if (outfile.is_open()) {
          outfile << "Performance Index (integrated L2 norm): " << performance_index << "\n";
          outfile.close();
          RCLCPP_INFO(this->get_logger(), "Performance index saved to performance_index.txt");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing performance index.");
        }
        rclcpp::shutdown();
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerformanceIndexNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
