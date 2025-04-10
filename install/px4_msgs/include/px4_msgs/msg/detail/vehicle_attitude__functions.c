// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleAttitude.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__VehicleAttitude__init(px4_msgs__msg__VehicleAttitude * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // q
  // delta_q_reset
  // quat_reset_counter
  return true;
}

void
px4_msgs__msg__VehicleAttitude__fini(px4_msgs__msg__VehicleAttitude * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // q
  // delta_q_reset
  // quat_reset_counter
}

bool
px4_msgs__msg__VehicleAttitude__are_equal(const px4_msgs__msg__VehicleAttitude * lhs, const px4_msgs__msg__VehicleAttitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // timestamp_sample
  if (lhs->timestamp_sample != rhs->timestamp_sample) {
    return false;
  }
  // q
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->q[i] != rhs->q[i]) {
      return false;
    }
  }
  // delta_q_reset
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->delta_q_reset[i] != rhs->delta_q_reset[i]) {
      return false;
    }
  }
  // quat_reset_counter
  if (lhs->quat_reset_counter != rhs->quat_reset_counter) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__VehicleAttitude__copy(
  const px4_msgs__msg__VehicleAttitude * input,
  px4_msgs__msg__VehicleAttitude * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // q
  for (size_t i = 0; i < 4; ++i) {
    output->q[i] = input->q[i];
  }
  // delta_q_reset
  for (size_t i = 0; i < 4; ++i) {
    output->delta_q_reset[i] = input->delta_q_reset[i];
  }
  // quat_reset_counter
  output->quat_reset_counter = input->quat_reset_counter;
  return true;
}

px4_msgs__msg__VehicleAttitude *
px4_msgs__msg__VehicleAttitude__create()
{
  px4_msgs__msg__VehicleAttitude * msg = (px4_msgs__msg__VehicleAttitude *)malloc(sizeof(px4_msgs__msg__VehicleAttitude));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleAttitude));
  bool success = px4_msgs__msg__VehicleAttitude__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleAttitude__destroy(px4_msgs__msg__VehicleAttitude * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleAttitude__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleAttitude__Sequence__init(px4_msgs__msg__VehicleAttitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleAttitude * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleAttitude *)calloc(size, sizeof(px4_msgs__msg__VehicleAttitude));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleAttitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleAttitude__fini(&data[i - 1]);
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
px4_msgs__msg__VehicleAttitude__Sequence__fini(px4_msgs__msg__VehicleAttitude__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleAttitude__fini(&array->data[i]);
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

px4_msgs__msg__VehicleAttitude__Sequence *
px4_msgs__msg__VehicleAttitude__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleAttitude__Sequence * array = (px4_msgs__msg__VehicleAttitude__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleAttitude__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleAttitude__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleAttitude__Sequence__destroy(px4_msgs__msg__VehicleAttitude__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleAttitude__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__VehicleAttitude__Sequence__are_equal(const px4_msgs__msg__VehicleAttitude__Sequence * lhs, const px4_msgs__msg__VehicleAttitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__VehicleAttitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__VehicleAttitude__Sequence__copy(
  const px4_msgs__msg__VehicleAttitude__Sequence * input,
  px4_msgs__msg__VehicleAttitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__VehicleAttitude);
    px4_msgs__msg__VehicleAttitude * data =
      (px4_msgs__msg__VehicleAttitude *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__VehicleAttitude__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__VehicleAttitude__fini(&data[i]);
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
    if (!px4_msgs__msg__VehicleAttitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
