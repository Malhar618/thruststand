// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/EstimatorEventFlags.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__TRAITS_HPP_

#include "px4_msgs/msg/detail/estimator_event_flags__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::EstimatorEventFlags & msg,
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

  // member: information_event_changes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "information_event_changes: ";
    value_to_yaml(msg.information_event_changes, out);
    out << "\n";
  }

  // member: gps_checks_passed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_checks_passed: ";
    value_to_yaml(msg.gps_checks_passed, out);
    out << "\n";
  }

  // member: reset_vel_to_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_vel_to_gps: ";
    value_to_yaml(msg.reset_vel_to_gps, out);
    out << "\n";
  }

  // member: reset_vel_to_flow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_vel_to_flow: ";
    value_to_yaml(msg.reset_vel_to_flow, out);
    out << "\n";
  }

  // member: reset_vel_to_vision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_vel_to_vision: ";
    value_to_yaml(msg.reset_vel_to_vision, out);
    out << "\n";
  }

  // member: reset_vel_to_zero
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_vel_to_zero: ";
    value_to_yaml(msg.reset_vel_to_zero, out);
    out << "\n";
  }

  // member: reset_pos_to_last_known
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_pos_to_last_known: ";
    value_to_yaml(msg.reset_pos_to_last_known, out);
    out << "\n";
  }

  // member: reset_pos_to_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_pos_to_gps: ";
    value_to_yaml(msg.reset_pos_to_gps, out);
    out << "\n";
  }

  // member: reset_pos_to_vision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_pos_to_vision: ";
    value_to_yaml(msg.reset_pos_to_vision, out);
    out << "\n";
  }

  // member: starting_gps_fusion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "starting_gps_fusion: ";
    value_to_yaml(msg.starting_gps_fusion, out);
    out << "\n";
  }

  // member: starting_vision_pos_fusion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "starting_vision_pos_fusion: ";
    value_to_yaml(msg.starting_vision_pos_fusion, out);
    out << "\n";
  }

  // member: starting_vision_vel_fusion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "starting_vision_vel_fusion: ";
    value_to_yaml(msg.starting_vision_vel_fusion, out);
    out << "\n";
  }

  // member: starting_vision_yaw_fusion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "starting_vision_yaw_fusion: ";
    value_to_yaml(msg.starting_vision_yaw_fusion, out);
    out << "\n";
  }

  // member: yaw_aligned_to_imu_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_aligned_to_imu_gps: ";
    value_to_yaml(msg.yaw_aligned_to_imu_gps, out);
    out << "\n";
  }

  // member: reset_hgt_to_baro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_hgt_to_baro: ";
    value_to_yaml(msg.reset_hgt_to_baro, out);
    out << "\n";
  }

  // member: reset_hgt_to_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_hgt_to_gps: ";
    value_to_yaml(msg.reset_hgt_to_gps, out);
    out << "\n";
  }

  // member: reset_hgt_to_rng
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_hgt_to_rng: ";
    value_to_yaml(msg.reset_hgt_to_rng, out);
    out << "\n";
  }

  // member: reset_hgt_to_ev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_hgt_to_ev: ";
    value_to_yaml(msg.reset_hgt_to_ev, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::EstimatorEventFlags & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::EstimatorEventFlags>()
{
  return "px4_msgs::msg::EstimatorEventFlags";
}

template<>
inline const char * name<px4_msgs::msg::EstimatorEventFlags>()
{
  return "px4_msgs/msg/EstimatorEventFlags";
}

template<>
struct has_fixed_size<px4_msgs::msg::EstimatorEventFlags>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::EstimatorEventFlags>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::EstimatorEventFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__TRAITS_HPP_
