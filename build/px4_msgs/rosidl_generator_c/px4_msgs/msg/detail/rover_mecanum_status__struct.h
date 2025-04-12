// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverMecanumStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RoverMecanumStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverMecanumStatus
{
  uint64_t timestamp;
  float measured_forward_speed;
  float adjusted_forward_speed_setpoint;
  float measured_lateral_speed;
  float adjusted_lateral_speed_setpoint;
  float measured_yaw_rate;
  float clyaw_yaw_rate_setpoint;
  float adjusted_yaw_rate_setpoint;
  float measured_yaw;
  float adjusted_yaw_setpoint;
  float pid_yaw_rate_integral;
  float pid_yaw_integral;
  float pid_forward_throttle_integral;
  float pid_lateral_throttle_integral;
} px4_msgs__msg__RoverMecanumStatus;

// Struct for a sequence of px4_msgs__msg__RoverMecanumStatus.
typedef struct px4_msgs__msg__RoverMecanumStatus__Sequence
{
  px4_msgs__msg__RoverMecanumStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverMecanumStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_
