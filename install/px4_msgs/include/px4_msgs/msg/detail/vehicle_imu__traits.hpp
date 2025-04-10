// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/VehicleImu.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__VEHICLE_IMU__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__VEHICLE_IMU__TRAITS_HPP_

#include "px4_msgs/msg/detail/vehicle_imu__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::VehicleImu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: timestamp_sample
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_sample: ";
    value_to_yaml(msg.timestamp_sample, out);
    out << "\n";
  }

  // member: accel_device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_device_id: ";
    value_to_yaml(msg.accel_device_id, out);
    out << "\n";
  }

  // member: gyro_device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_device_id: ";
    value_to_yaml(msg.gyro_device_id, out);
    out << "\n";
  }

  // member: delta_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.delta_angle.size() == 0) {
      out << "delta_angle: []\n";
    } else {
      out << "delta_angle:\n";
      for (auto item : msg.delta_angle) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: delta_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.delta_velocity.size() == 0) {
      out << "delta_velocity: []\n";
    } else {
      out << "delta_velocity:\n";
      for (auto item : msg.delta_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: delta_angle_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_angle_dt: ";
    value_to_yaml(msg.delta_angle_dt, out);
    out << "\n";
  }

  // member: delta_velocity_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_velocity_dt: ";
    value_to_yaml(msg.delta_velocity_dt, out);
    out << "\n";
  }

  // member: delta_angle_clipping
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_angle_clipping: ";
    value_to_yaml(msg.delta_angle_clipping, out);
    out << "\n";
  }

  // member: delta_velocity_clipping
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_velocity_clipping: ";
    value_to_yaml(msg.delta_velocity_clipping, out);
    out << "\n";
  }

  // member: accel_calibration_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_calibration_count: ";
    value_to_yaml(msg.accel_calibration_count, out);
    out << "\n";
  }

  // member: gyro_calibration_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_calibration_count: ";
    value_to_yaml(msg.gyro_calibration_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::VehicleImu & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::VehicleImu>()
{
  return "px4_msgs::msg::VehicleImu";
}

template<>
inline const char * name<px4_msgs::msg::VehicleImu>()
{
  return "px4_msgs/msg/VehicleImu";
}

template<>
struct has_fixed_size<px4_msgs::msg::VehicleImu>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::VehicleImu>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::VehicleImu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__VEHICLE_IMU__TRAITS_HPP_
