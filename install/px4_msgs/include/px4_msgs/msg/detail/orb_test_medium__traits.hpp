// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/OrbTestMedium.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ORB_TEST_MEDIUM__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ORB_TEST_MEDIUM__TRAITS_HPP_

#include "px4_msgs/msg/detail/orb_test_medium__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::OrbTestMedium & msg,
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

  // member: val
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "val: ";
    value_to_yaml(msg.val, out);
    out << "\n";
  }

  // member: junk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.junk.size() == 0) {
      out << "junk: []\n";
    } else {
      out << "junk:\n";
      for (auto item : msg.junk) {
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

inline std::string to_yaml(const px4_msgs::msg::OrbTestMedium & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::OrbTestMedium>()
{
  return "px4_msgs::msg::OrbTestMedium";
}

template<>
inline const char * name<px4_msgs::msg::OrbTestMedium>()
{
  return "px4_msgs/msg/OrbTestMedium";
}

template<>
struct has_fixed_size<px4_msgs::msg::OrbTestMedium>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::OrbTestMedium>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::OrbTestMedium>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ORB_TEST_MEDIUM__TRAITS_HPP_
