// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverAckermannSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RoverAckermannSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__RoverAckermannSetpoint
{
  uint64_t timestamp;
  float forward_speed_setpoint;
  float forward_speed_setpoint_normalized;
  float steering_setpoint;
  float steering_setpoint_normalized;
  float lateral_acceleration_setpoint;
} px4_msgs__msg__RoverAckermannSetpoint;

// Struct for a sequence of px4_msgs__msg__RoverAckermannSetpoint.
typedef struct px4_msgs__msg__RoverAckermannSetpoint__Sequence
{
  px4_msgs__msg__RoverAckermannSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverAckermannSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__STRUCT_H_
