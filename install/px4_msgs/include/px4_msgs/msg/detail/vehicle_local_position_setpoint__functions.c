// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleLocalPositionSetpoint.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_local_position_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__VehicleLocalPositionSetpoint__init(px4_msgs__msg__VehicleLocalPositionSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // x
  // y
  // z
  // vx
  // vy
  // vz
  // acceleration
  // thrust
  // yaw
  // yawspeed
  return true;
}

void
px4_msgs__msg__VehicleLocalPositionSetpoint__fini(px4_msgs__msg__VehicleLocalPositionSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // x
  // y
  // z
  // vx
  // vy
  // vz
  // acceleration
  // thrust
  // yaw
  // yawspeed
}

bool
px4_msgs__msg__VehicleLocalPositionSetpoint__are_equal(const px4_msgs__msg__VehicleLocalPositionSetpoint * lhs, const px4_msgs__msg__VehicleLocalPositionSetpoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  // vz
  if (lhs->vz != rhs->vz) {
    return false;
  }
  // acceleration
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->acceleration[i] != rhs->acceleration[i]) {
      return false;
    }
  }
  // thrust
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->thrust[i] != rhs->thrust[i]) {
      return false;
    }
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // yawspeed
  if (lhs->yawspeed != rhs->yawspeed) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__VehicleLocalPositionSetpoint__copy(
  const px4_msgs__msg__VehicleLocalPositionSetpoint * input,
  px4_msgs__msg__VehicleLocalPositionSetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  // vz
  output->vz = input->vz;
  // acceleration
  for (size_t i = 0; i < 3; ++i) {
    output->acceleration[i] = input->acceleration[i];
  }
  // thrust
  for (size_t i = 0; i < 3; ++i) {
    output->thrust[i] = input->thrust[i];
  }
  // yaw
  output->yaw = input->yaw;
  // yawspeed
  output->yawspeed = input->yawspeed;
  return true;
}

px4_msgs__msg__VehicleLocalPositionSetpoint *
px4_msgs__msg__VehicleLocalPositionSetpoint__create()
{
  px4_msgs__msg__VehicleLocalPositionSetpoint * msg = (px4_msgs__msg__VehicleLocalPositionSetpoint *)malloc(sizeof(px4_msgs__msg__VehicleLocalPositionSetpoint));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleLocalPositionSetpoint));
  bool success = px4_msgs__msg__VehicleLocalPositionSetpoint__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleLocalPositionSetpoint__destroy(px4_msgs__msg__VehicleLocalPositionSetpoint * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleLocalPositionSetpoint__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__init(px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleLocalPositionSetpoint * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleLocalPositionSetpoint *)calloc(size, sizeof(px4_msgs__msg__VehicleLocalPositionSetpoint));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleLocalPositionSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleLocalPositionSetpoint__fini(&data[i - 1]);
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
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__fini(px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleLocalPositionSetpoint__fini(&array->data[i]);
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

px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence *
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * array = (px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__destroy(px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__are_equal(const px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * lhs, const px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__VehicleLocalPositionSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence__copy(
  const px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * input,
  px4_msgs__msg__VehicleLocalPositionSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__VehicleLocalPositionSetpoint);
    px4_msgs__msg__VehicleLocalPositionSetpoint * data =
      (px4_msgs__msg__VehicleLocalPositionSetpoint *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__VehicleLocalPositionSetpoint__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__VehicleLocalPositionSetpoint__fini(&data[i]);
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
    if (!px4_msgs__msg__VehicleLocalPositionSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
