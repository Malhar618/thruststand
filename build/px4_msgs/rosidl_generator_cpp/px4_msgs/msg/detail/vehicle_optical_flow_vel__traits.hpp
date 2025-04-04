// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/VehicleOpticalFlowVel.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__VEHICLE_OPTICAL_FLOW_VEL__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__VEHICLE_OPTICAL_FLOW_VEL__TRAITS_HPP_

#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::VehicleOpticalFlowVel & msg,
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

  // member: vel_body
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_body.size() == 0) {
      out << "vel_body: []\n";
    } else {
      out << "vel_body:\n";
      for (auto item : msg.vel_body) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_ne
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_ne.size() == 0) {
      out << "vel_ne: []\n";
    } else {
      out << "vel_ne:\n";
      for (auto item : msg.vel_ne) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_body_filtered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_body_filtered.size() == 0) {
      out << "vel_body_filtered: []\n";
    } else {
      out << "vel_body_filtered:\n";
      for (auto item : msg.vel_body_filtered) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_ne_filtered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_ne_filtered.size() == 0) {
      out << "vel_ne_filtered: []\n";
    } else {
      out << "vel_ne_filtered:\n";
      for (auto item : msg.vel_ne_filtered) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: flow_rate_uncompensated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.flow_rate_uncompensated.size() == 0) {
      out << "flow_rate_uncompensated: []\n";
    } else {
      out << "flow_rate_uncompensated:\n";
      for (auto item : msg.flow_rate_uncompensated) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: flow_rate_compensated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.flow_rate_compensated.size() == 0) {
      out << "flow_rate_compensated: []\n";
    } else {
      out << "flow_rate_compensated:\n";
      for (auto item : msg.flow_rate_compensated) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gyro_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gyro_rate.size() == 0) {
      out << "gyro_rate: []\n";
    } else {
      out << "gyro_rate:\n";
      for (auto item : msg.gyro_rate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gyro_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gyro_bias.size() == 0) {
      out << "gyro_bias: []\n";
    } else {
      out << "gyro_bias:\n";
      for (auto item : msg.gyro_bias) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ref_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ref_gyro.size() == 0) {
      out << "ref_gyro: []\n";
    } else {
      out << "ref_gyro:\n";
      for (auto item : msg.ref_gyro) {
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

inline std::string to_yaml(const px4_msgs::msg::VehicleOpticalFlowVel & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::VehicleOpticalFlowVel>()
{
  return "px4_msgs::msg::VehicleOpticalFlowVel";
}

template<>
inline const char * name<px4_msgs::msg::VehicleOpticalFlowVel>()
{
  return "px4_msgs/msg/VehicleOpticalFlowVel";
}

template<>
struct has_fixed_size<px4_msgs::msg::VehicleOpticalFlowVel>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::VehicleOpticalFlowVel>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::VehicleOpticalFlowVel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__VEHICLE_OPTICAL_FLOW_VEL__TRAITS_HPP_
