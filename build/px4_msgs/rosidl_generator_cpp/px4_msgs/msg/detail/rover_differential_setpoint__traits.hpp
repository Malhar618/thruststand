// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverDifferentialSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_

#include "px4_msgs/msg/detail/rover_differential_setpoint__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::RoverDifferentialSetpoint & msg,
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

  // member: forward_speed_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_speed_setpoint: ";
    value_to_yaml(msg.forward_speed_setpoint, out);
    out << "\n";
  }

  // member: forward_speed_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_speed_setpoint_normalized: ";
    value_to_yaml(msg.forward_speed_setpoint_normalized, out);
    out << "\n";
  }

  // member: yaw_rate_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate_setpoint: ";
    value_to_yaml(msg.yaw_rate_setpoint, out);
    out << "\n";
  }

  // member: speed_diff_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_diff_setpoint_normalized: ";
    value_to_yaml(msg.speed_diff_setpoint_normalized, out);
    out << "\n";
  }

  // member: yaw_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_setpoint: ";
    value_to_yaml(msg.yaw_setpoint, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::RoverDifferentialSetpoint & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::RoverDifferentialSetpoint>()
{
  return "px4_msgs::msg::RoverDifferentialSetpoint";
}

template<>
inline const char * name<px4_msgs::msg::RoverDifferentialSetpoint>()
{
  return "px4_msgs/msg/RoverDifferentialSetpoint";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_
