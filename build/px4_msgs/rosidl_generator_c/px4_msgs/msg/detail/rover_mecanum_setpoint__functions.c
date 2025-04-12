// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/RoverMecanumSetpoint.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_mecanum_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__RoverMecanumSetpoint__init(px4_msgs__msg__RoverMecanumSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // forward_speed_setpoint
  // forward_speed_setpoint_normalized
  // lateral_speed_setpoint
  // lateral_speed_setpoint_normalized
  // yaw_rate_setpoint
  // speed_diff_setpoint_normalized
  // yaw_setpoint
  return true;
}

void
px4_msgs__msg__RoverMecanumSetpoint__fini(px4_msgs__msg__RoverMecanumSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // forward_speed_setpoint
  // forward_speed_setpoint_normalized
  // lateral_speed_setpoint
  // lateral_speed_setpoint_normalized
  // yaw_rate_setpoint
  // speed_diff_setpoint_normalized
  // yaw_setpoint
}

bool
px4_msgs__msg__RoverMecanumSetpoint__are_equal(const px4_msgs__msg__RoverMecanumSetpoint * lhs, const px4_msgs__msg__RoverMecanumSetpoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // forward_speed_setpoint
  if (lhs->forward_speed_setpoint != rhs->forward_speed_setpoint) {
    return false;
  }
  // forward_speed_setpoint_normalized
  if (lhs->forward_speed_setpoint_normalized != rhs->forward_speed_setpoint_normalized) {
    return false;
  }
  // lateral_speed_setpoint
  if (lhs->lateral_speed_setpoint != rhs->lateral_speed_setpoint) {
    return false;
  }
  // lateral_speed_setpoint_normalized
  if (lhs->lateral_speed_setpoint_normalized != rhs->lateral_speed_setpoint_normalized) {
    return false;
  }
  // yaw_rate_setpoint
  if (lhs->yaw_rate_setpoint != rhs->yaw_rate_setpoint) {
    return false;
  }
  // speed_diff_setpoint_normalized
  if (lhs->speed_diff_setpoint_normalized != rhs->speed_diff_setpoint_normalized) {
    return false;
  }
  // yaw_setpoint
  if (lhs->yaw_setpoint != rhs->yaw_setpoint) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__RoverMecanumSetpoint__copy(
  const px4_msgs__msg__RoverMecanumSetpoint * input,
  px4_msgs__msg__RoverMecanumSetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // forward_speed_setpoint
  output->forward_speed_setpoint = input->forward_speed_setpoint;
  // forward_speed_setpoint_normalized
  output->forward_speed_setpoint_normalized = input->forward_speed_setpoint_normalized;
  // lateral_speed_setpoint
  output->lateral_speed_setpoint = input->lateral_speed_setpoint;
  // lateral_speed_setpoint_normalized
  output->lateral_speed_setpoint_normalized = input->lateral_speed_setpoint_normalized;
  // yaw_rate_setpoint
  output->yaw_rate_setpoint = input->yaw_rate_setpoint;
  // speed_diff_setpoint_normalized
  output->speed_diff_setpoint_normalized = input->speed_diff_setpoint_normalized;
  // yaw_setpoint
  output->yaw_setpoint = input->yaw_setpoint;
  return true;
}

px4_msgs__msg__RoverMecanumSetpoint *
px4_msgs__msg__RoverMecanumSetpoint__create()
{
  px4_msgs__msg__RoverMecanumSetpoint * msg = (px4_msgs__msg__RoverMecanumSetpoint *)malloc(sizeof(px4_msgs__msg__RoverMecanumSetpoint));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__RoverMecanumSetpoint));
  bool success = px4_msgs__msg__RoverMecanumSetpoint__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__RoverMecanumSetpoint__destroy(px4_msgs__msg__RoverMecanumSetpoint * msg)
{
  if (msg) {
    px4_msgs__msg__RoverMecanumSetpoint__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__RoverMecanumSetpoint__Sequence__init(px4_msgs__msg__RoverMecanumSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__RoverMecanumSetpoint * data = NULL;
  if (size) {
    data = (px4_msgs__msg__RoverMecanumSetpoint *)calloc(size, sizeof(px4_msgs__msg__RoverMecanumSetpoint));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__RoverMecanumSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__RoverMecanumSetpoint__fini(&data[i - 1]);
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
px4_msgs__msg__RoverMecanumSetpoint__Sequence__fini(px4_msgs__msg__RoverMecanumSetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__RoverMecanumSetpoint__fini(&array->data[i]);
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

px4_msgs__msg__RoverMecanumSetpoint__Sequence *
px4_msgs__msg__RoverMecanumSetpoint__Sequence__create(size_t size)
{
  px4_msgs__msg__RoverMecanumSetpoint__Sequence * array = (px4_msgs__msg__RoverMecanumSetpoint__Sequence *)malloc(sizeof(px4_msgs__msg__RoverMecanumSetpoint__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__RoverMecanumSetpoint__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__RoverMecanumSetpoint__Sequence__destroy(px4_msgs__msg__RoverMecanumSetpoint__Sequence * array)
{
  if (array) {
    px4_msgs__msg__RoverMecanumSetpoint__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__RoverMecanumSetpoint__Sequence__are_equal(const px4_msgs__msg__RoverMecanumSetpoint__Sequence * lhs, const px4_msgs__msg__RoverMecanumSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__RoverMecanumSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__RoverMecanumSetpoint__Sequence__copy(
  const px4_msgs__msg__RoverMecanumSetpoint__Sequence * input,
  px4_msgs__msg__RoverMecanumSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__RoverMecanumSetpoint);
    px4_msgs__msg__RoverMecanumSetpoint * data =
      (px4_msgs__msg__RoverMecanumSetpoint *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__RoverMecanumSetpoint__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__RoverMecanumSetpoint__fini(&data[i]);
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
    if (!px4_msgs__msg__RoverMecanumSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
