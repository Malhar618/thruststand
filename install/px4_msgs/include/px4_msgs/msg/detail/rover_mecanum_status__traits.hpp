// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverMecanumStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__TRAITS_HPP_

#include "px4_msgs/msg/detail/rover_mecanum_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::RoverMecanumStatus & msg,
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

  // member: measured_lateral_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_lateral_speed: ";
    value_to_yaml(msg.measured_lateral_speed, out);
    out << "\n";
  }

  // member: adjusted_lateral_speed_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_lateral_speed_setpoint: ";
    value_to_yaml(msg.adjusted_lateral_speed_setpoint, out);
    out << "\n";
  }

  // member: measured_yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_yaw_rate: ";
    value_to_yaml(msg.measured_yaw_rate, out);
    out << "\n";
  }

  // member: clyaw_yaw_rate_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clyaw_yaw_rate_setpoint: ";
    value_to_yaml(msg.clyaw_yaw_rate_setpoint, out);
    out << "\n";
  }

  // member: adjusted_yaw_rate_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_yaw_rate_setpoint: ";
    value_to_yaml(msg.adjusted_yaw_rate_setpoint, out);
    out << "\n";
  }

  // member: measured_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_yaw: ";
    value_to_yaml(msg.measured_yaw, out);
    out << "\n";
  }

  // member: adjusted_yaw_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_yaw_setpoint: ";
    value_to_yaml(msg.adjusted_yaw_setpoint, out);
    out << "\n";
  }

  // member: pid_yaw_rate_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_yaw_rate_integral: ";
    value_to_yaml(msg.pid_yaw_rate_integral, out);
    out << "\n";
  }

  // member: pid_yaw_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_yaw_integral: ";
    value_to_yaml(msg.pid_yaw_integral, out);
    out << "\n";
  }

  // member: pid_forward_throttle_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_forward_throttle_integral: ";
    value_to_yaml(msg.pid_forward_throttle_integral, out);
    out << "\n";
  }

  // member: pid_lateral_throttle_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_lateral_throttle_integral: ";
    value_to_yaml(msg.pid_lateral_throttle_integral, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::RoverMecanumStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::RoverMecanumStatus>()
{
  return "px4_msgs::msg::RoverMecanumStatus";
}

template<>
inline const char * name<px4_msgs::msg::RoverMecanumStatus>()
{
  return "px4_msgs/msg/RoverMecanumStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverMecanumStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverMecanumStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverMecanumStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__TRAITS_HPP_
