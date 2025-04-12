// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/Mission.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__MISSION__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__MISSION__TRAITS_HPP_

#include "px4_msgs/msg/detail/mission__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::Mission & msg,
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

  // member: mission_dataman_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_dataman_id: ";
    value_to_yaml(msg.mission_dataman_id, out);
    out << "\n";
  }

  // member: fence_dataman_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fence_dataman_id: ";
    value_to_yaml(msg.fence_dataman_id, out);
    out << "\n";
  }

  // member: safepoint_dataman_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safepoint_dataman_id: ";
    value_to_yaml(msg.safepoint_dataman_id, out);
    out << "\n";
  }

  // member: count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "count: ";
    value_to_yaml(msg.count, out);
    out << "\n";
  }

  // member: current_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_seq: ";
    value_to_yaml(msg.current_seq, out);
    out << "\n";
  }

  // member: land_start_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "land_start_index: ";
    value_to_yaml(msg.land_start_index, out);
    out << "\n";
  }

  // member: land_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "land_index: ";
    value_to_yaml(msg.land_index, out);
    out << "\n";
  }

  // member: mission_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_id: ";
    value_to_yaml(msg.mission_id, out);
    out << "\n";
  }

  // member: geofence_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geofence_id: ";
    value_to_yaml(msg.geofence_id, out);
    out << "\n";
  }

  // member: safe_points_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_points_id: ";
    value_to_yaml(msg.safe_points_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::Mission & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::Mission>()
{
  return "px4_msgs::msg::Mission";
}

template<>
inline const char * name<px4_msgs::msg::Mission>()
{
  return "px4_msgs/msg/Mission";
}

template<>
struct has_fixed_size<px4_msgs::msg::Mission>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::Mission>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::Mission>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__MISSION__TRAITS_HPP_
