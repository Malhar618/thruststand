// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/VehicleAngularAccelerationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_ACCELERATION_SETPOINT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_ACCELERATION_SETPOINT__TRAITS_HPP_

#include "px4_msgs/msg/detail/vehicle_angular_acceleration_setpoint__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::VehicleAngularAccelerationSetpoint & msg,
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

  // member: xyz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.xyz.size() == 0) {
      out << "xyz: []\n";
    } else {
      out << "xyz:\n";
      for (auto item : msg.xyz) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::VehicleAngularAccelerationSetpoint & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::VehicleAngularAccelerationSetpoint>()
{
  return "px4_msgs::msg::VehicleAngularAccelerationSetpoint";
}

template<>
inline const char * name<px4_msgs::msg::VehicleAngularAccelerationSetpoint>()
{
  return "px4_msgs/msg/VehicleAngularAccelerationSetpoint";
}

template<>
struct has_fixed_size<px4_msgs::msg::VehicleAngularAccelerationSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::VehicleAngularAccelerationSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::VehicleAngularAccelerationSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__VEHICLE_ANGULAR_ACCELERATION_SETPOINT__TRAITS_HPP_
