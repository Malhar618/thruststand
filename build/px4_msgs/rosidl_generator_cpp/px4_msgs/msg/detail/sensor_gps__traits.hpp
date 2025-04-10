// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/SensorGps.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__SENSOR_GPS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__SENSOR_GPS__TRAITS_HPP_

#include "px4_msgs/msg/detail/sensor_gps__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const px4_msgs::msg::SensorGps & msg,
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

  // member: device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_id: ";
    value_to_yaml(msg.device_id, out);
    out << "\n";
  }

  // member: latitude_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude_deg: ";
    value_to_yaml(msg.latitude_deg, out);
    out << "\n";
  }

  // member: longitude_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude_deg: ";
    value_to_yaml(msg.longitude_deg, out);
    out << "\n";
  }

  // member: altitude_msl_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude_msl_m: ";
    value_to_yaml(msg.altitude_msl_m, out);
    out << "\n";
  }

  // member: altitude_ellipsoid_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude_ellipsoid_m: ";
    value_to_yaml(msg.altitude_ellipsoid_m, out);
    out << "\n";
  }

  // member: s_variance_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_variance_m_s: ";
    value_to_yaml(msg.s_variance_m_s, out);
    out << "\n";
  }

  // member: c_variance_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c_variance_rad: ";
    value_to_yaml(msg.c_variance_rad, out);
    out << "\n";
  }

  // member: fix_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_type: ";
    value_to_yaml(msg.fix_type, out);
    out << "\n";
  }

  // member: eph
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eph: ";
    value_to_yaml(msg.eph, out);
    out << "\n";
  }

  // member: epv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "epv: ";
    value_to_yaml(msg.epv, out);
    out << "\n";
  }

  // member: hdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hdop: ";
    value_to_yaml(msg.hdop, out);
    out << "\n";
  }

  // member: vdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vdop: ";
    value_to_yaml(msg.vdop, out);
    out << "\n";
  }

  // member: noise_per_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "noise_per_ms: ";
    value_to_yaml(msg.noise_per_ms, out);
    out << "\n";
  }

  // member: automatic_gain_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "automatic_gain_control: ";
    value_to_yaml(msg.automatic_gain_control, out);
    out << "\n";
  }

  // member: jamming_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jamming_state: ";
    value_to_yaml(msg.jamming_state, out);
    out << "\n";
  }

  // member: jamming_indicator
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jamming_indicator: ";
    value_to_yaml(msg.jamming_indicator, out);
    out << "\n";
  }

  // member: spoofing_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spoofing_state: ";
    value_to_yaml(msg.spoofing_state, out);
    out << "\n";
  }

  // member: vel_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_m_s: ";
    value_to_yaml(msg.vel_m_s, out);
    out << "\n";
  }

  // member: vel_n_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_n_m_s: ";
    value_to_yaml(msg.vel_n_m_s, out);
    out << "\n";
  }

  // member: vel_e_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_e_m_s: ";
    value_to_yaml(msg.vel_e_m_s, out);
    out << "\n";
  }

  // member: vel_d_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_d_m_s: ";
    value_to_yaml(msg.vel_d_m_s, out);
    out << "\n";
  }

  // member: cog_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cog_rad: ";
    value_to_yaml(msg.cog_rad, out);
    out << "\n";
  }

  // member: vel_ned_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_ned_valid: ";
    value_to_yaml(msg.vel_ned_valid, out);
    out << "\n";
  }

  // member: timestamp_time_relative
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_time_relative: ";
    value_to_yaml(msg.timestamp_time_relative, out);
    out << "\n";
  }

  // member: time_utc_usec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_utc_usec: ";
    value_to_yaml(msg.time_utc_usec, out);
    out << "\n";
  }

  // member: satellites_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "satellites_used: ";
    value_to_yaml(msg.satellites_used, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: heading_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_offset: ";
    value_to_yaml(msg.heading_offset, out);
    out << "\n";
  }

  // member: heading_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_accuracy: ";
    value_to_yaml(msg.heading_accuracy, out);
    out << "\n";
  }

  // member: rtcm_injection_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtcm_injection_rate: ";
    value_to_yaml(msg.rtcm_injection_rate, out);
    out << "\n";
  }

  // member: selected_rtcm_instance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selected_rtcm_instance: ";
    value_to_yaml(msg.selected_rtcm_instance, out);
    out << "\n";
  }

  // member: rtcm_crc_failed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtcm_crc_failed: ";
    value_to_yaml(msg.rtcm_crc_failed, out);
    out << "\n";
  }

  // member: rtcm_msg_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtcm_msg_used: ";
    value_to_yaml(msg.rtcm_msg_used, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const px4_msgs::msg::SensorGps & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<px4_msgs::msg::SensorGps>()
{
  return "px4_msgs::msg::SensorGps";
}

template<>
inline const char * name<px4_msgs::msg::SensorGps>()
{
  return "px4_msgs/msg/SensorGps";
}

template<>
struct has_fixed_size<px4_msgs::msg::SensorGps>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::SensorGps>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::SensorGps>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__SENSOR_GPS__TRAITS_HPP_
