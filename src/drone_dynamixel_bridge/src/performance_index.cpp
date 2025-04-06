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
    mission_duration_sec_(30.0), // Adjust mission duration as needed
    actual_received_(false),
    desired_received_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing PerformanceIndexNode...");

    // Subscribe to the actual attitude topic
    auto qos = rclcpp::QoS(10).best_effort();
    actual_sub_ = this->create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&PerformanceIndexNode::actualAttitudeCallback, this, std::placeholders::_1)
    );

    // Subscribe to the desired attitude setpoint topic
    desired_sub_ = this->create_subscription<VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", qos,
      std::bind(&PerformanceIndexNode::desiredAttitudeCallback, this, std::placeholders::_1)
    );

    // Use a timer to periodically compute the error and integrate it (10 Hz)
    timer_ = this->create_wall_timer(100ms, std::bind(&PerformanceIndexNode::timerCallback, this));
    prev_time_ = this->now();
  }

private:
  // Subscribers
  rclcpp::Subscription<VehicleAttitude>::SharedPtr actual_sub_;
  rclcpp::Subscription<VehicleAttitudeSetpoint>::SharedPtr desired_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Mutex to protect shared data
  std::mutex mutex_;

  // Latest measured and desired Euler angles (in radians: roll, pitch, yaw)
  Eigen::Vector3d actual_attitude_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_attitude_{Eigen::Vector3d::Zero()};

  // Flags to check whether messages have been received
  bool actual_received_;
  bool desired_received_;

  // For performance index calculation
  double accumulated_error_sq_;
  rclcpp::Time prev_time_;
  rclcpp::Time mission_start_time_;
  bool mission_started_ = false;
  double mission_duration_sec_;  // mission duration in seconds

  // Callback for actual attitude (VehicleAttitude)
  void actualAttitudeCallback(const VehicleAttitude::SharedPtr msg)
  {
    tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); // PX4 uses [w, x, y, z]
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

  // Callback for desired attitude setpoint (VehicleAttitudeSetpoint)
  // We assume the setpoint message provides desired roll, pitch, yaw in radians in fields: roll_body, pitch_body, yaw_body
  void desiredAttitudeCallback(const VehicleAttitudeSetpoint::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      desired_attitude_ = Eigen::Vector3d(msg->roll_body, msg->pitch_body, msg->yaw_body);
      desired_received_ = true;
    }
  }

  // Timer callback runs at 10 Hz to compute the instantaneous error and integrate
  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    // Only compute error if both actual and desired values have been received
    bool compute_error = false;
    Eigen::Vector3d actual, desired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (actual_received_ && desired_received_) {
        actual = actual_attitude_;
        desired = desired_attitude_;
        compute_error = true;
      }
    }

    if (compute_error) {
      // Error = desired - actual
      Eigen::Vector3d error = desired - actual;
      // Accumulate squared error (weighted by time step)
      accumulated_error_sq_ += error.squaredNorm() * dt;
      double elapsed = (now - mission_start_time_).seconds();

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Elapsed=%.2fs, L2 Error=%.4f", elapsed, std::sqrt(error.squaredNorm())
      );

      // If mission duration has elapsed, compute the performance index
      if (elapsed >= mission_duration_sec_) {
        double performance_index = std::sqrt(accumulated_error_sq_);
        RCLCPP_INFO(this->get_logger(),
          "Mission complete. Performance Index (integrated L2 norm) = %.4f", performance_index);

        // Write performance index to a text file
        std::ofstream outfile("performance_index.txt");
        if (outfile.is_open()) {
          outfile << "Performance Index (integrated L2 norm): " << performance_index << "\n";
          outfile.close();
          RCLCPP_INFO(this->get_logger(), "Performance index saved to performance_index.txt");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing performance index.");
        }
        // Shut down the node after saving result (or you can choose to continue logging)
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
