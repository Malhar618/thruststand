// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/YawEstimatorStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/yaw_estimator_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__YawEstimatorStatus__init(px4_msgs__msg__YawEstimatorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // yaw_composite
  // yaw_variance
  // yaw_composite_valid
  // yaw
  // innov_vn
  // innov_ve
  // weight
  return true;
}

void
px4_msgs__msg__YawEstimatorStatus__fini(px4_msgs__msg__YawEstimatorStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // yaw_composite
  // yaw_variance
  // yaw_composite_valid
  // yaw
  // innov_vn
  // innov_ve
  // weight
}

bool
px4_msgs__msg__YawEstimatorStatus__are_equal(const px4_msgs__msg__YawEstimatorStatus * lhs, const px4_msgs__msg__YawEstimatorStatus * rhs)
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
  // yaw_composite
  if (lhs->yaw_composite != rhs->yaw_composite) {
    return false;
  }
  // yaw_variance
  if (lhs->yaw_variance != rhs->yaw_variance) {
    return false;
  }
  // yaw_composite_valid
  if (lhs->yaw_composite_valid != rhs->yaw_composite_valid) {
    return false;
  }
  // yaw
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->yaw[i] != rhs->yaw[i]) {
      return false;
    }
  }
  // innov_vn
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->innov_vn[i] != rhs->innov_vn[i]) {
      return false;
    }
  }
  // innov_ve
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->innov_ve[i] != rhs->innov_ve[i]) {
      return false;
    }
  }
  // weight
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->weight[i] != rhs->weight[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__YawEstimatorStatus__copy(
  const px4_msgs__msg__YawEstimatorStatus * input,
  px4_msgs__msg__YawEstimatorStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // yaw_composite
  output->yaw_composite = input->yaw_composite;
  // yaw_variance
  output->yaw_variance = input->yaw_variance;
  // yaw_composite_valid
  output->yaw_composite_valid = input->yaw_composite_valid;
  // yaw
  for (size_t i = 0; i < 5; ++i) {
    output->yaw[i] = input->yaw[i];
  }
  // innov_vn
  for (size_t i = 0; i < 5; ++i) {
    output->innov_vn[i] = input->innov_vn[i];
  }
  // innov_ve
  for (size_t i = 0; i < 5; ++i) {
    output->innov_ve[i] = input->innov_ve[i];
  }
  // weight
  for (size_t i = 0; i < 5; ++i) {
    output->weight[i] = input->weight[i];
  }
  return true;
}

px4_msgs__msg__YawEstimatorStatus *
px4_msgs__msg__YawEstimatorStatus__create()
{
  px4_msgs__msg__YawEstimatorStatus * msg = (px4_msgs__msg__YawEstimatorStatus *)malloc(sizeof(px4_msgs__msg__YawEstimatorStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__YawEstimatorStatus));
  bool success = px4_msgs__msg__YawEstimatorStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__YawEstimatorStatus__destroy(px4_msgs__msg__YawEstimatorStatus * msg)
{
  if (msg) {
    px4_msgs__msg__YawEstimatorStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__YawEstimatorStatus__Sequence__init(px4_msgs__msg__YawEstimatorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__YawEstimatorStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__YawEstimatorStatus *)calloc(size, sizeof(px4_msgs__msg__YawEstimatorStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__YawEstimatorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__YawEstimatorStatus__fini(&data[i - 1]);
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
px4_msgs__msg__YawEstimatorStatus__Sequence__fini(px4_msgs__msg__YawEstimatorStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__YawEstimatorStatus__fini(&array->data[i]);
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

px4_msgs__msg__YawEstimatorStatus__Sequence *
px4_msgs__msg__YawEstimatorStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__YawEstimatorStatus__Sequence * array = (px4_msgs__msg__YawEstimatorStatus__Sequence *)malloc(sizeof(px4_msgs__msg__YawEstimatorStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__YawEstimatorStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__YawEstimatorStatus__Sequence__destroy(px4_msgs__msg__YawEstimatorStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__YawEstimatorStatus__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__YawEstimatorStatus__Sequence__are_equal(const px4_msgs__msg__YawEstimatorStatus__Sequence * lhs, const px4_msgs__msg__YawEstimatorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__YawEstimatorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__YawEstimatorStatus__Sequence__copy(
  const px4_msgs__msg__YawEstimatorStatus__Sequence * input,
  px4_msgs__msg__YawEstimatorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__YawEstimatorStatus);
    px4_msgs__msg__YawEstimatorStatus * data =
      (px4_msgs__msg__YawEstimatorStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__YawEstimatorStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__YawEstimatorStatus__fini(&data[i]);
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
    if (!px4_msgs__msg__YawEstimatorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
