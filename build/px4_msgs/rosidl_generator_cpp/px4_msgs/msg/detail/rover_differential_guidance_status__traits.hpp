// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverDifferentialGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__TRAITS_HPP_

#include "px4_msgs/msg/detail/rover_differential_guidance_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::RoverDifferentialGuidanceStatus & msg,
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

  // member: heading_error_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_error_deg: ";
    value_to_yaml(msg.heading_error_deg, out);
    out << "\n";
  }

  // member: state_machine
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_machine: ";
    value_to_yaml(msg.state_machine, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::RoverDifferentialGuidanceStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::RoverDifferentialGuidanceStatus>()
{
  return "px4_msgs::msg::RoverDifferentialGuidanceStatus";
}

template<>
inline const char * name<px4_msgs::msg::RoverDifferentialGuidanceStatus>()
{
  return "px4_msgs/msg/RoverDifferentialGuidanceStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverDifferentialGuidanceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverDifferentialGuidanceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverDifferentialGuidanceStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__TRAITS_HPP_
