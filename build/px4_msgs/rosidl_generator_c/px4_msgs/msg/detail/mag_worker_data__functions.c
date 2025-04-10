// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/MagWorkerData.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/mag_worker_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__MagWorkerData__init(px4_msgs__msg__MagWorkerData * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // done_count
  // calibration_points_perside
  // calibration_interval_perside_us
  // calibration_counter_total
  // side_data_collected
  // x
  // y
  // z
  return true;
}

void
px4_msgs__msg__MagWorkerData__fini(px4_msgs__msg__MagWorkerData * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // done_count
  // calibration_points_perside
  // calibration_interval_perside_us
  // calibration_counter_total
  // side_data_collected
  // x
  // y
  // z
}

bool
px4_msgs__msg__MagWorkerData__are_equal(const px4_msgs__msg__MagWorkerData * lhs, const px4_msgs__msg__MagWorkerData * rhs)
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
  // done_count
  if (lhs->done_count != rhs->done_count) {
    return false;
  }
  // calibration_points_perside
  if (lhs->calibration_points_perside != rhs->calibration_points_perside) {
    return false;
  }
  // calibration_interval_perside_us
  if (lhs->calibration_interval_perside_us != rhs->calibration_interval_perside_us) {
    return false;
  }
  // calibration_counter_total
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->calibration_counter_total[i] != rhs->calibration_counter_total[i]) {
      return false;
    }
  }
  // side_data_collected
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->side_data_collected[i] != rhs->side_data_collected[i]) {
      return false;
    }
  }
  // x
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->x[i] != rhs->x[i]) {
      return false;
    }
  }
  // y
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->y[i] != rhs->y[i]) {
      return false;
    }
  }
  // z
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->z[i] != rhs->z[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__MagWorkerData__copy(
  const px4_msgs__msg__MagWorkerData * input,
  px4_msgs__msg__MagWorkerData * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // done_count
  output->done_count = input->done_count;
  // calibration_points_perside
  output->calibration_points_perside = input->calibration_points_perside;
  // calibration_interval_perside_us
  output->calibration_interval_perside_us = input->calibration_interval_perside_us;
  // calibration_counter_total
  for (size_t i = 0; i < 4; ++i) {
    output->calibration_counter_total[i] = input->calibration_counter_total[i];
  }
  // side_data_collected
  for (size_t i = 0; i < 4; ++i) {
    output->side_data_collected[i] = input->side_data_collected[i];
  }
  // x
  for (size_t i = 0; i < 4; ++i) {
    output->x[i] = input->x[i];
  }
  // y
  for (size_t i = 0; i < 4; ++i) {
    output->y[i] = input->y[i];
  }
  // z
  for (size_t i = 0; i < 4; ++i) {
    output->z[i] = input->z[i];
  }
  return true;
}

px4_msgs__msg__MagWorkerData *
px4_msgs__msg__MagWorkerData__create()
{
  px4_msgs__msg__MagWorkerData * msg = (px4_msgs__msg__MagWorkerData *)malloc(sizeof(px4_msgs__msg__MagWorkerData));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__MagWorkerData));
  bool success = px4_msgs__msg__MagWorkerData__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__MagWorkerData__destroy(px4_msgs__msg__MagWorkerData * msg)
{
  if (msg) {
    px4_msgs__msg__MagWorkerData__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__MagWorkerData__Sequence__init(px4_msgs__msg__MagWorkerData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__MagWorkerData * data = NULL;
  if (size) {
    data = (px4_msgs__msg__MagWorkerData *)calloc(size, sizeof(px4_msgs__msg__MagWorkerData));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__MagWorkerData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__MagWorkerData__fini(&data[i - 1]);
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
px4_msgs__msg__MagWorkerData__Sequence__fini(px4_msgs__msg__MagWorkerData__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__MagWorkerData__fini(&array->data[i]);
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

px4_msgs__msg__MagWorkerData__Sequence *
px4_msgs__msg__MagWorkerData__Sequence__create(size_t size)
{
  px4_msgs__msg__MagWorkerData__Sequence * array = (px4_msgs__msg__MagWorkerData__Sequence *)malloc(sizeof(px4_msgs__msg__MagWorkerData__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__MagWorkerData__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__MagWorkerData__Sequence__destroy(px4_msgs__msg__MagWorkerData__Sequence * array)
{
  if (array) {
    px4_msgs__msg__MagWorkerData__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__MagWorkerData__Sequence__are_equal(const px4_msgs__msg__MagWorkerData__Sequence * lhs, const px4_msgs__msg__MagWorkerData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__MagWorkerData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__MagWorkerData__Sequence__copy(
  const px4_msgs__msg__MagWorkerData__Sequence * input,
  px4_msgs__msg__MagWorkerData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__MagWorkerData);
    px4_msgs__msg__MagWorkerData * data =
      (px4_msgs__msg__MagWorkerData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__MagWorkerData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__MagWorkerData__fini(&data[i]);
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
    if (!px4_msgs__msg__MagWorkerData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
