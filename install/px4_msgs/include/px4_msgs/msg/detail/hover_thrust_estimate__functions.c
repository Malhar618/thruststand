// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/HoverThrustEstimate.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/hover_thrust_estimate__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__HoverThrustEstimate__init(px4_msgs__msg__HoverThrustEstimate * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // hover_thrust
  // hover_thrust_var
  // accel_innov
  // accel_innov_var
  // accel_innov_test_ratio
  // accel_noise_var
  // valid
  return true;
}

void
px4_msgs__msg__HoverThrustEstimate__fini(px4_msgs__msg__HoverThrustEstimate * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // hover_thrust
  // hover_thrust_var
  // accel_innov
  // accel_innov_var
  // accel_innov_test_ratio
  // accel_noise_var
  // valid
}

bool
px4_msgs__msg__HoverThrustEstimate__are_equal(const px4_msgs__msg__HoverThrustEstimate * lhs, const px4_msgs__msg__HoverThrustEstimate * rhs)
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
  // hover_thrust
  if (lhs->hover_thrust != rhs->hover_thrust) {
    return false;
  }
  // hover_thrust_var
  if (lhs->hover_thrust_var != rhs->hover_thrust_var) {
    return false;
  }
  // accel_innov
  if (lhs->accel_innov != rhs->accel_innov) {
    return false;
  }
  // accel_innov_var
  if (lhs->accel_innov_var != rhs->accel_innov_var) {
    return false;
  }
  // accel_innov_test_ratio
  if (lhs->accel_innov_test_ratio != rhs->accel_innov_test_ratio) {
    return false;
  }
  // accel_noise_var
  if (lhs->accel_noise_var != rhs->accel_noise_var) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__HoverThrustEstimate__copy(
  const px4_msgs__msg__HoverThrustEstimate * input,
  px4_msgs__msg__HoverThrustEstimate * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // hover_thrust
  output->hover_thrust = input->hover_thrust;
  // hover_thrust_var
  output->hover_thrust_var = input->hover_thrust_var;
  // accel_innov
  output->accel_innov = input->accel_innov;
  // accel_innov_var
  output->accel_innov_var = input->accel_innov_var;
  // accel_innov_test_ratio
  output->accel_innov_test_ratio = input->accel_innov_test_ratio;
  // accel_noise_var
  output->accel_noise_var = input->accel_noise_var;
  // valid
  output->valid = input->valid;
  return true;
}

px4_msgs__msg__HoverThrustEstimate *
px4_msgs__msg__HoverThrustEstimate__create()
{
  px4_msgs__msg__HoverThrustEstimate * msg = (px4_msgs__msg__HoverThrustEstimate *)malloc(sizeof(px4_msgs__msg__HoverThrustEstimate));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__HoverThrustEstimate));
  bool success = px4_msgs__msg__HoverThrustEstimate__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__HoverThrustEstimate__destroy(px4_msgs__msg__HoverThrustEstimate * msg)
{
  if (msg) {
    px4_msgs__msg__HoverThrustEstimate__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__HoverThrustEstimate__Sequence__init(px4_msgs__msg__HoverThrustEstimate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__HoverThrustEstimate * data = NULL;
  if (size) {
    data = (px4_msgs__msg__HoverThrustEstimate *)calloc(size, sizeof(px4_msgs__msg__HoverThrustEstimate));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__HoverThrustEstimate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__HoverThrustEstimate__fini(&data[i - 1]);
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
px4_msgs__msg__HoverThrustEstimate__Sequence__fini(px4_msgs__msg__HoverThrustEstimate__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__HoverThrustEstimate__fini(&array->data[i]);
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

px4_msgs__msg__HoverThrustEstimate__Sequence *
px4_msgs__msg__HoverThrustEstimate__Sequence__create(size_t size)
{
  px4_msgs__msg__HoverThrustEstimate__Sequence * array = (px4_msgs__msg__HoverThrustEstimate__Sequence *)malloc(sizeof(px4_msgs__msg__HoverThrustEstimate__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__HoverThrustEstimate__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__HoverThrustEstimate__Sequence__destroy(px4_msgs__msg__HoverThrustEstimate__Sequence * array)
{
  if (array) {
    px4_msgs__msg__HoverThrustEstimate__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__HoverThrustEstimate__Sequence__are_equal(const px4_msgs__msg__HoverThrustEstimate__Sequence * lhs, const px4_msgs__msg__HoverThrustEstimate__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__HoverThrustEstimate__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__HoverThrustEstimate__Sequence__copy(
  const px4_msgs__msg__HoverThrustEstimate__Sequence * input,
  px4_msgs__msg__HoverThrustEstimate__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__HoverThrustEstimate);
    px4_msgs__msg__HoverThrustEstimate * data =
      (px4_msgs__msg__HoverThrustEstimate *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__HoverThrustEstimate__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__HoverThrustEstimate__fini(&data[i]);
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
    if (!px4_msgs__msg__HoverThrustEstimate__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
