#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip> // For std::setprecision, std::fixed
#include <cmath>   // For M_PI, atan2, asin
#include <ctime>   // For filename timestamp
#include <sstream> // For filename timestamp formatting
#include <filesystem> // For creating directories (optional but good practice)
#include <optional> // To store latest messages
#include <mutex>    // To protect access to shared data

// Conversion factor from radians to degrees
const double RAD_TO_DEG = 180.0 / M_PI;

class AngularStateLogger : public rclcpp::Node
{
public:
    AngularStateLogger() : Node("angular_state_logger")
    {
        // --- File Setup ---
        // Create a unique filename based on the current time
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "log_%Y%m%d_%H%M%S.csv");
        log_filename_ = ss.str();

        // Optional: Create a directory for logs if it doesn't exist
        std::filesystem::path log_dir("logs");
        try {
            if (!std::filesystem::exists(log_dir)) {
                std::filesystem::create_directory(log_dir);
            }
            log_filepath_ = log_dir / log_filename_;
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s. Saving in current directory.", e.what());
            log_filepath_ = log_filename_; // Fallback to current directory
        }


        // Open the log file
        log_file_.open(log_filepath_);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_filepath_.c_str());
            // Optionally, throw an exception or signal an error state
            rclcpp::shutdown(); // Stop the node if logging isn't possible
            return;
        }

        // Write the CSV header
        log_file_ << "px4_timestamp_us,system_timestamp_ns,"
                  << "roll_deg,pitch_deg,yaw_deg,"
                  << "roll_rate_dps,pitch_rate_dps,yaw_rate_dps" << std::endl;

        RCLCPP_INFO(this->get_logger(), "Logging angular state to: %s", log_filepath_.c_str());

        // --- ROS Setup ---
        // Quality of Service profile - match PX4 bridge (usually reliable, history depth 10)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // Subscribers
        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&AngularStateLogger::attitude_callback, this, std::placeholders::_1));

        angular_velocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos,
            std::bind(&AngularStateLogger::angular_velocity_callback, this, std::placeholders::_1));

        // Timer for periodic logging (e.g., at 10 Hz)
        logging_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz logging rate
            std::bind(&AngularStateLogger::log_data, this));
    }

    ~AngularStateLogger() // Destructor to ensure file is closed
    {
        if (log_file_.is_open()) {
            log_file_.close();
            RCLCPP_INFO(this->get_logger(), "Log file closed: %s", log_filepath_.c_str());
        }
    }

private:
    // --- Data Storage ---
    std::mutex data_mutex_; // Protects access to latest_attitude_ and latest_angular_velocity_
    std::optional<px4_msgs::msg::VehicleAttitude> latest_attitude_;
    std::optional<px4_msgs::msg::VehicleAngularVelocity> latest_angular_velocity_;

    // --- File Handling ---
    std::ofstream log_file_;
    std::string log_filename_;
    std::filesystem::path log_filepath_;


    // --- ROS Components ---
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
    rclcpp::TimerBase::SharedPtr logging_timer_;

    // --- Callbacks ---
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_attitude_ = *msg; // Store the latest message
    }

    void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_angular_velocity_ = *msg; // Store the latest message
    }

    // --- Helper Functions ---
    // Function to convert quaternion to Euler angles (Roll, Pitch, Yaw in degrees)
    // Standard ZYX rotation sequence (Yaw, Pitch, Roll) common in aerospace
    void quaternion_to_euler(const std::array<float, 4>& q, double& roll_deg, double& pitch_deg, double& yaw_deg)
    {
        // q[0] = w, q[1] = x, q[2] = y, q[3] = z
        double w = q[0];
        double x = q[1];
        double y = q[2];
        double z = q[3];

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll_deg = std::atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1) {
            pitch_deg = std::copysign(M_PI / 2, sinp) * RAD_TO_DEG; // Use 90 degrees if out of range
        } else {
            pitch_deg = std::asin(sinp) * RAD_TO_DEG;
        }

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw_deg = std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    }


    // --- Logging Function (called by timer) ---
    void log_data()
    {
        // Lock the mutex to safely access the stored messages
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Check if we have received both types of messages at least once
        if (!latest_attitude_.has_value() || !latest_angular_velocity_.has_value()) {
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for initial attitude and angular velocity messages...");
            return; // Don't log until we have both
        }

        // Get system timestamp
        auto system_now_ns = this->get_clock()->now().nanoseconds();

        // Extract data from the latest messages
        uint64_t attitude_ts_us = latest_attitude_->timestamp;
        uint64_t angular_vel_ts_us = latest_angular_velocity_->timestamp;

        // --- Attitude ---
        double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
        quaternion_to_euler(latest_attitude_->q, roll_deg, pitch_deg, yaw_deg);

        // --- Angular Velocity ---
        // The message provides xyz rates directly in rad/s
        double roll_rate_dps = latest_angular_velocity_->xyz[0] * RAD_TO_DEG;
        double pitch_rate_dps = latest_angular_velocity_->xyz[1] * RAD_TO_DEG;
        double yaw_rate_dps = latest_angular_velocity_->xyz[2] * RAD_TO_DEG;

        // --- Write to File ---
        if (log_file_.is_open()) {
            // Use the timestamp from the angular velocity message as the primary PX4 timestamp,
            // as it often updates more frequently or is used for control. You could also average or log both.
            log_file_ << angular_vel_ts_us << ","
                      << system_now_ns << ","
                      << std::fixed << std::setprecision(4) // Format floating point numbers
                      << roll_deg << "," << pitch_deg << "," << yaw_deg << ","
                      << roll_rate_dps << "," << pitch_rate_dps << "," << yaw_rate_dps
                      << std::endl;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Log file is not open. Cannot write data.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AngularStateLogger>();
    if (node != nullptr) { // Basic check if node creation failed (e.g., file error)
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
