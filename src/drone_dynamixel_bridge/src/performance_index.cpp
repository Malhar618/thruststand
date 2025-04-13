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
    mission_duration_sec_(45.0), // Mission duration
    actual_received_(false),
    desired_received_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing PerformanceIndexNode...");

    // Subscribe to actual vehicle attitude from PX4
    auto qos_actual = rclcpp::QoS(10).best_effort();
    actual_sub_ = this->create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_actual,
      std::bind(&PerformanceIndexNode::actualAttitudeCallback, this, std::placeholders::_1)
    );

    // Subscribe to desired vehicle attitude setpoint
    auto qos_desired = rclcpp::QoS(10).best_effort();
    desired_sub_ = this->create_subscription<VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", qos_desired, // Often published by the controller
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
    tf2::Quaternion q_actual(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); // x,y,z,w
    tf2::Matrix3x3 m_actual(q_actual);
    double roll, pitch, yaw;
    m_actual.getRPY(roll, pitch, yaw);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      actual_attitude_ = Eigen::Vector3d(roll, pitch, yaw);
      actual_received_ = true;
      // Start mission timer on first valid actual attitude message
      if (!mission_started_ && actual_received_ && desired_received_) { // Wait for both to start timer
        mission_start_time_ = this->now();
        mission_started_ = true;
         RCLCPP_INFO(this->get_logger(), "Mission Timer Started.");
      }
    }
  }

  // Callback for desired attitude setpoint
  void desiredAttitudeCallback(const VehicleAttitudeSetpoint::SharedPtr msg)
  {
    // *** FIXED: Convert desired quaternion q_d to Euler angles ***
    // PX4 Attitude Setpoint quaternion q_d is [w, x, y, z]
    tf2::Quaternion q_desired(msg->q_d[1], msg->q_d[2], msg->q_d[3], msg->q_d[0]); // x,y,z,w
    tf2::Matrix3x3 m_desired(q_desired);
    double roll_des, pitch_des, yaw_des;
    m_desired.getRPY(roll_des, pitch_des, yaw_des);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      desired_attitude_ = Eigen::Vector3d(roll_des, pitch_des, yaw_des);
      desired_received_ = true;
       // Start mission timer on first valid desired attitude message (if actual already received)
      if (!mission_started_ && actual_received_ && desired_received_) {
        mission_start_time_ = this->now();
        mission_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Mission Timer Started.");
      }
    }
  }

  // Timer callback: integrates squared error over time and computes performance index at mission end
  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 0) { // Prevent issues with non-positive dt
         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Non-positive dt detected (%.4f), skipping error accumulation.", dt);
        prev_time_ = now; // Still update time
        return;
    }
    prev_time_ = now;

    Eigen::Vector3d actual, desired;
    bool compute_error = false;
    bool mission_has_started = false; // Local copy to avoid holding mutex too long

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Only proceed if mission has started and we have both actual and desired data
      if (mission_started_ && actual_received_ && desired_received_) {
        actual = actual_attitude_;
        desired = desired_attitude_;
        compute_error = true;
        mission_has_started = mission_started_; // Copy state
      }
    }

    if (compute_error && mission_has_started) {
      // Compute error vector in Euler angles
      Eigen::Vector3d error = desired - actual;
      // Handle yaw wrap-around (-pi to pi)
      error[2] = atan2(sin(error[2]), cos(error[2]));

      accumulated_error_sq_ += error.squaredNorm() * dt;
      double elapsed = (now - mission_start_time_).seconds();

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, // Log every 2 seconds
        "Elapsed: %.2fs / %.1fs, Inst Err (r,p,y): [%.3f, %.3f, %.3f], Inst L2: %.4f, Accum L2Sq: %.4f",
        elapsed, mission_duration_sec_,
        error[0], error[1], error[2], // Log individual errors
        std::sqrt(error.squaredNorm()), accumulated_error_sq_);

      // Once mission duration is reached, compute and write performance index
      if (elapsed >= mission_duration_sec_) {
        double performance_index = std::sqrt(accumulated_error_sq_);
        RCLCPP_INFO(this->get_logger(),
          "Mission complete. Performance Index (Integrated L2 Norm): %.6f", performance_index);

        // Write performance index to file
        std::ofstream outfile("performance_index.txt");
        if (outfile.is_open()) {
          outfile << "Performance Index (integrated L2 norm): " << performance_index << "\n";
          outfile.close();
          RCLCPP_INFO(this->get_logger(), "Performance index saved to performance_index.txt");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing performance index.");
        }
        rclcpp::shutdown(); // Shut down the node after completion
      }
    } else if (!mission_has_started) {
         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for mission to start (need both actual and desired attitude)...");
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
