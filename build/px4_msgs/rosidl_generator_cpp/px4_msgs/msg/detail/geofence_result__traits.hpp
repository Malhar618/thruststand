// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/GeofenceResult.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_

#include "px4_msgs/msg/detail/geofence_result__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::GeofenceResult & msg,
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

  // member: geofence_max_dist_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_max_dist_triggered: ";
    value_to_yaml(msg.geofence_max_dist_triggered, out);
    out << "\n";
  }

  // member: geofence_max_alt_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_max_alt_triggered: ";
    value_to_yaml(msg.geofence_max_alt_triggered, out);
    out << "\n";
  }

  // member: geofence_custom_fence_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_custom_fence_triggered: ";
    value_to_yaml(msg.geofence_custom_fence_triggered, out);
    out << "\n";
  }

  // member: geofence_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_action: ";
    value_to_yaml(msg.geofence_action, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::GeofenceResult & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::GeofenceResult>()
{
  return "px4_msgs::msg::GeofenceResult";
}

template<>
inline const char * name<px4_msgs::msg::GeofenceResult>()
{
  return "px4_msgs/msg/GeofenceResult";
}

template<>
struct has_fixed_size<px4_msgs::msg::GeofenceResult>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::GeofenceResult>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::GeofenceResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__GEOFENCE_RESULT__TRAITS_HPP_
