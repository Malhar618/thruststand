// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/EstimatorAidSource3d.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_aid_source3d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__EstimatorAidSource3d__init(px4_msgs__msg__EstimatorAidSource3d * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // estimator_instance
  // device_id
  // time_last_fuse
  // observation
  // observation_variance
  // innovation
  // innovation_filtered
  // innovation_variance
  // test_ratio
  // test_ratio_filtered
  // innovation_rejected
  // fused
  return true;
}

void
px4_msgs__msg__EstimatorAidSource3d__fini(px4_msgs__msg__EstimatorAidSource3d * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // estimator_instance
  // device_id
  // time_last_fuse
  // observation
  // observation_variance
  // innovation
  // innovation_filtered
  // innovation_variance
  // test_ratio
  // test_ratio_filtered
  // innovation_rejected
  // fused
}

bool
px4_msgs__msg__EstimatorAidSource3d__are_equal(const px4_msgs__msg__EstimatorAidSource3d * lhs, const px4_msgs__msg__EstimatorAidSource3d * rhs)
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
  // estimator_instance
  if (lhs->estimator_instance != rhs->estimator_instance) {
    return false;
  }
  // device_id
  if (lhs->device_id != rhs->device_id) {
    return false;
  }
  // time_last_fuse
  if (lhs->time_last_fuse != rhs->time_last_fuse) {
    return false;
  }
  // observation
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->observation[i] != rhs->observation[i]) {
      return false;
    }
  }
  // observation_variance
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->observation_variance[i] != rhs->observation_variance[i]) {
      return false;
    }
  }
  // innovation
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->innovation[i] != rhs->innovation[i]) {
      return false;
    }
  }
  // innovation_filtered
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->innovation_filtered[i] != rhs->innovation_filtered[i]) {
      return false;
    }
  }
  // innovation_variance
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->innovation_variance[i] != rhs->innovation_variance[i]) {
      return false;
    }
  }
  // test_ratio
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->test_ratio[i] != rhs->test_ratio[i]) {
      return false;
    }
  }
  // test_ratio_filtered
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->test_ratio_filtered[i] != rhs->test_ratio_filtered[i]) {
      return false;
    }
  }
  // innovation_rejected
  if (lhs->innovation_rejected != rhs->innovation_rejected) {
    return false;
  }
  // fused
  if (lhs->fused != rhs->fused) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__EstimatorAidSource3d__copy(
  const px4_msgs__msg__EstimatorAidSource3d * input,
  px4_msgs__msg__EstimatorAidSource3d * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // estimator_instance
  output->estimator_instance = input->estimator_instance;
  // device_id
  output->device_id = input->device_id;
  // time_last_fuse
  output->time_last_fuse = input->time_last_fuse;
  // observation
  for (size_t i = 0; i < 3; ++i) {
    output->observation[i] = input->observation[i];
  }
  // observation_variance
  for (size_t i = 0; i < 3; ++i) {
    output->observation_variance[i] = input->observation_variance[i];
  }
  // innovation
  for (size_t i = 0; i < 3; ++i) {
    output->innovation[i] = input->innovation[i];
  }
  // innovation_filtered
  for (size_t i = 0; i < 3; ++i) {
    output->innovation_filtered[i] = input->innovation_filtered[i];
  }
  // innovation_variance
  for (size_t i = 0; i < 3; ++i) {
    output->innovation_variance[i] = input->innovation_variance[i];
  }
  // test_ratio
  for (size_t i = 0; i < 3; ++i) {
    output->test_ratio[i] = input->test_ratio[i];
  }
  // test_ratio_filtered
  for (size_t i = 0; i < 3; ++i) {
    output->test_ratio_filtered[i] = input->test_ratio_filtered[i];
  }
  // innovation_rejected
  output->innovation_rejected = input->innovation_rejected;
  // fused
  output->fused = input->fused;
  return true;
}

px4_msgs__msg__EstimatorAidSource3d *
px4_msgs__msg__EstimatorAidSource3d__create()
{
  px4_msgs__msg__EstimatorAidSource3d * msg = (px4_msgs__msg__EstimatorAidSource3d *)malloc(sizeof(px4_msgs__msg__EstimatorAidSource3d));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__EstimatorAidSource3d));
  bool success = px4_msgs__msg__EstimatorAidSource3d__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__EstimatorAidSource3d__destroy(px4_msgs__msg__EstimatorAidSource3d * msg)
{
  if (msg) {
    px4_msgs__msg__EstimatorAidSource3d__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__EstimatorAidSource3d__Sequence__init(px4_msgs__msg__EstimatorAidSource3d__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__EstimatorAidSource3d * data = NULL;
  if (size) {
    data = (px4_msgs__msg__EstimatorAidSource3d *)calloc(size, sizeof(px4_msgs__msg__EstimatorAidSource3d));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__EstimatorAidSource3d__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__EstimatorAidSource3d__fini(&data[i - 1]);
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
px4_msgs__msg__EstimatorAidSource3d__Sequence__fini(px4_msgs__msg__EstimatorAidSource3d__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__EstimatorAidSource3d__fini(&array->data[i]);
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

px4_msgs__msg__EstimatorAidSource3d__Sequence *
px4_msgs__msg__EstimatorAidSource3d__Sequence__create(size_t size)
{
  px4_msgs__msg__EstimatorAidSource3d__Sequence * array = (px4_msgs__msg__EstimatorAidSource3d__Sequence *)malloc(sizeof(px4_msgs__msg__EstimatorAidSource3d__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__EstimatorAidSource3d__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__EstimatorAidSource3d__Sequence__destroy(px4_msgs__msg__EstimatorAidSource3d__Sequence * array)
{
  if (array) {
    px4_msgs__msg__EstimatorAidSource3d__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__EstimatorAidSource3d__Sequence__are_equal(const px4_msgs__msg__EstimatorAidSource3d__Sequence * lhs, const px4_msgs__msg__EstimatorAidSource3d__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__EstimatorAidSource3d__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__EstimatorAidSource3d__Sequence__copy(
  const px4_msgs__msg__EstimatorAidSource3d__Sequence * input,
  px4_msgs__msg__EstimatorAidSource3d__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__EstimatorAidSource3d);
    px4_msgs__msg__EstimatorAidSource3d * data =
      (px4_msgs__msg__EstimatorAidSource3d *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__EstimatorAidSource3d__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__EstimatorAidSource3d__fini(&data[i]);
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
    if (!px4_msgs__msg__EstimatorAidSource3d__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
