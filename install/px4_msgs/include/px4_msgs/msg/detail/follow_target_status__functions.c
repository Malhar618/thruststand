// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/FollowTargetStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/follow_target_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__FollowTargetStatus__init(px4_msgs__msg__FollowTargetStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // tracked_target_course
  // follow_angle
  // orbit_angle_setpoint
  // angular_rate_setpoint
  // desired_position_raw
  // in_emergency_ascent
  // gimbal_pitch
  return true;
}

void
px4_msgs__msg__FollowTargetStatus__fini(px4_msgs__msg__FollowTargetStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // tracked_target_course
  // follow_angle
  // orbit_angle_setpoint
  // angular_rate_setpoint
  // desired_position_raw
  // in_emergency_ascent
  // gimbal_pitch
}

bool
px4_msgs__msg__FollowTargetStatus__are_equal(const px4_msgs__msg__FollowTargetStatus * lhs, const px4_msgs__msg__FollowTargetStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // tracked_target_course
  if (lhs->tracked_target_course != rhs->tracked_target_course) {
    return false;
  }
  // follow_angle
  if (lhs->follow_angle != rhs->follow_angle) {
    return false;
  }
  // orbit_angle_setpoint
  if (lhs->orbit_angle_setpoint != rhs->orbit_angle_setpoint) {
    return false;
  }
  // angular_rate_setpoint
  if (lhs->angular_rate_setpoint != rhs->angular_rate_setpoint) {
    return false;
  }
  // desired_position_raw
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->desired_position_raw[i] != rhs->desired_position_raw[i]) {
      return false;
    }
  }
  // in_emergency_ascent
  if (lhs->in_emergency_ascent != rhs->in_emergency_ascent) {
    return false;
  }
  // gimbal_pitch
  if (lhs->gimbal_pitch != rhs->gimbal_pitch) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__FollowTargetStatus__copy(
  const px4_msgs__msg__FollowTargetStatus * input,
  px4_msgs__msg__FollowTargetStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // tracked_target_course
  output->tracked_target_course = input->tracked_target_course;
  // follow_angle
  output->follow_angle = input->follow_angle;
  // orbit_angle_setpoint
  output->orbit_angle_setpoint = input->orbit_angle_setpoint;
  // angular_rate_setpoint
  output->angular_rate_setpoint = input->angular_rate_setpoint;
  // desired_position_raw
  for (size_t i = 0; i < 3; ++i) {
    output->desired_position_raw[i] = input->desired_position_raw[i];
  }
  // in_emergency_ascent
  output->in_emergency_ascent = input->in_emergency_ascent;
  // gimbal_pitch
  output->gimbal_pitch = input->gimbal_pitch;
  return true;
}

px4_msgs__msg__FollowTargetStatus *
px4_msgs__msg__FollowTargetStatus__create()
{
  px4_msgs__msg__FollowTargetStatus * msg = (px4_msgs__msg__FollowTargetStatus *)malloc(sizeof(px4_msgs__msg__FollowTargetStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__FollowTargetStatus));
  bool success = px4_msgs__msg__FollowTargetStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__FollowTargetStatus__destroy(px4_msgs__msg__FollowTargetStatus * msg)
{
  if (msg) {
    px4_msgs__msg__FollowTargetStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__FollowTargetStatus__Sequence__init(px4_msgs__msg__FollowTargetStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__FollowTargetStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__FollowTargetStatus *)calloc(size, sizeof(px4_msgs__msg__FollowTargetStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__FollowTargetStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__FollowTargetStatus__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__FollowTargetStatus__Sequence__fini(px4_msgs__msg__FollowTargetStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__FollowTargetStatus__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__FollowTargetStatus__Sequence *
px4_msgs__msg__FollowTargetStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__FollowTargetStatus__Sequence * array = (px4_msgs__msg__FollowTargetStatus__Sequence *)malloc(sizeof(px4_msgs__msg__FollowTargetStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__FollowTargetStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__FollowTargetStatus__Sequence__destroy(px4_msgs__msg__FollowTargetStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__FollowTargetStatus__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__FollowTargetStatus__Sequence__are_equal(const px4_msgs__msg__FollowTargetStatus__Sequence * lhs, const px4_msgs__msg__FollowTargetStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__FollowTargetStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__FollowTargetStatus__Sequence__copy(
  const px4_msgs__msg__FollowTargetStatus__Sequence * input,
  px4_msgs__msg__FollowTargetStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__FollowTargetStatus);
    px4_msgs__msg__FollowTargetStatus * data =
      (px4_msgs__msg__FollowTargetStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__FollowTargetStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__FollowTargetStatus__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__FollowTargetStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
