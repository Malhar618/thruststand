// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverAckermannGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RoverAckermannGuidanceStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverAckermannGuidanceStatus
{
  uint64_t timestamp;
  float lookahead_distance;
  float heading_error;
} px4_msgs__msg__RoverAckermannGuidanceStatus;

// Struct for a sequence of px4_msgs__msg__RoverAckermannGuidanceStatus.
typedef struct px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence
{
  px4_msgs__msg__RoverAckermannGuidanceStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__STRUCT_H_
