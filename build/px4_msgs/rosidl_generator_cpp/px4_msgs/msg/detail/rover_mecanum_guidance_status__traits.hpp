// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverMecanumGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_GUIDANCE_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_GUIDANCE_STATUS__TRAITS_HPP_

#include "px4_msgs/msg/detail/rover_mecanum_guidance_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::RoverMecanumGuidanceStatus & msg,
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

  // member: lookahead_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lookahead_distance: ";
    value_to_yaml(msg.lookahead_distance, out);
    out << "\n";
  }

  // member: heading_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_error: ";
    value_to_yaml(msg.heading_error, out);
    out << "\n";
  }

  // member: desired_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_speed: ";
    value_to_yaml(msg.desired_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::RoverMecanumGuidanceStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::RoverMecanumGuidanceStatus>()
{
  return "px4_msgs::msg::RoverMecanumGuidanceStatus";
}

template<>
inline const char * name<px4_msgs::msg::RoverMecanumGuidanceStatus>()
{
  return "px4_msgs/msg/RoverMecanumGuidanceStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverMecanumGuidanceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverMecanumGuidanceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverMecanumGuidanceStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_GUIDANCE_STATUS__TRAITS_HPP_
