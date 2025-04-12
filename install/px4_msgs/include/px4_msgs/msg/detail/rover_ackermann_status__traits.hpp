// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverAckermannStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__TRAITS_HPP_

#include "px4_msgs/msg/detail/rover_ackermann_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::RoverAckermannStatus & msg,
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

  // member: measured_forward_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_forward_speed: ";
    value_to_yaml(msg.measured_forward_speed, out);
    out << "\n";
  }

  // member: adjusted_forward_speed_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_forward_speed_setpoint: ";
    value_to_yaml(msg.adjusted_forward_speed_setpoint, out);
    out << "\n";
  }

  // member: steering_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_setpoint_normalized: ";
    value_to_yaml(msg.steering_setpoint_normalized, out);
    out << "\n";
  }

  // member: adjusted_steering_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_steering_setpoint_normalized: ";
    value_to_yaml(msg.adjusted_steering_setpoint_normalized, out);
    out << "\n";
  }

  // member: measured_lateral_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_lateral_acceleration: ";
    value_to_yaml(msg.measured_lateral_acceleration, out);
    out << "\n";
  }

  // member: pid_throttle_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_throttle_integral: ";
    value_to_yaml(msg.pid_throttle_integral, out);
    out << "\n";
  }

  // member: pid_lat_accel_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_lat_accel_integral: ";
    value_to_yaml(msg.pid_lat_accel_integral, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::RoverAckermannStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::RoverAckermannStatus>()
{
  return "px4_msgs::msg::RoverAckermannStatus";
}

template<>
inline const char * name<px4_msgs::msg::RoverAckermannStatus>()
{
  return "px4_msgs/msg/RoverAckermannStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverAckermannStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverAckermannStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverAckermannStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__TRAITS_HPP_
