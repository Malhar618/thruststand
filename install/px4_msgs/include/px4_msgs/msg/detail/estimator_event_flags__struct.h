// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/EstimatorEventFlags.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/EstimatorEventFlags in the package px4_msgs.
typedef struct px4_msgs__msg__EstimatorEventFlags
{
  uint64_t timestamp;
  uint64_t timestamp_sample;
  uint32_t information_event_changes;
  bool gps_checks_passed;
  bool reset_vel_to_gps;
  bool reset_vel_to_flow;
  bool reset_vel_to_vision;
  bool reset_vel_to_zero;
  bool reset_pos_to_last_known;
  bool reset_pos_to_gps;
  bool reset_pos_to_vision;
  bool starting_gps_fusion;
  bool starting_vision_pos_fusion;
  bool starting_vision_vel_fusion;
  bool starting_vision_yaw_fusion;
  bool yaw_aligned_to_imu_gps;
  bool reset_hgt_to_baro;
  bool reset_hgt_to_gps;
  bool reset_hgt_to_rng;
  bool reset_hgt_to_ev;
} px4_msgs__msg__EstimatorEventFlags;

// Struct for a sequence of px4_msgs__msg__EstimatorEventFlags.
typedef struct px4_msgs__msg__EstimatorEventFlags__Sequence
{
  px4_msgs__msg__EstimatorEventFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__EstimatorEventFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ESTIMATOR_EVENT_FLAGS__STRUCT_H_
