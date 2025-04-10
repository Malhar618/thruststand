// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorAccel.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_accel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorAccel__init(px4_msgs__msg__SensorAccel * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // x
  // y
  // z
  // temperature
  // error_count
  // clip_counter
  // samples
  return true;
}

void
px4_msgs__msg__SensorAccel__fini(px4_msgs__msg__SensorAccel * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // x
  // y
  // z
  // temperature
  // error_count
  // clip_counter
  // samples
}

bool
px4_msgs__msg__SensorAccel__are_equal(const px4_msgs__msg__SensorAccel * lhs, const px4_msgs__msg__SensorAccel * rhs)
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
  // device_id
  if (lhs->device_id != rhs->device_id) {
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
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // error_count
  if (lhs->error_count != rhs->error_count) {
    return false;
  }
  // clip_counter
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->clip_counter[i] != rhs->clip_counter[i]) {
      return false;
    }
  }
  // samples
  if (lhs->samples != rhs->samples) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__SensorAccel__copy(
  const px4_msgs__msg__SensorAccel * input,
  px4_msgs__msg__SensorAccel * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // device_id
  output->device_id = input->device_id;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // temperature
  output->temperature = input->temperature;
  // error_count
  output->error_count = input->error_count;
  // clip_counter
  for (size_t i = 0; i < 3; ++i) {
    output->clip_counter[i] = input->clip_counter[i];
  }
  // samples
  output->samples = input->samples;
  return true;
}

px4_msgs__msg__SensorAccel *
px4_msgs__msg__SensorAccel__create()
{
  px4_msgs__msg__SensorAccel * msg = (px4_msgs__msg__SensorAccel *)malloc(sizeof(px4_msgs__msg__SensorAccel));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorAccel));
  bool success = px4_msgs__msg__SensorAccel__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorAccel__destroy(px4_msgs__msg__SensorAccel * msg)
{
  if (msg) {
    px4_msgs__msg__SensorAccel__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorAccel__Sequence__init(px4_msgs__msg__SensorAccel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorAccel * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorAccel *)calloc(size, sizeof(px4_msgs__msg__SensorAccel));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorAccel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorAccel__fini(&data[i - 1]);
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
px4_msgs__msg__SensorAccel__Sequence__fini(px4_msgs__msg__SensorAccel__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorAccel__fini(&array->data[i]);
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

px4_msgs__msg__SensorAccel__Sequence *
px4_msgs__msg__SensorAccel__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorAccel__Sequence * array = (px4_msgs__msg__SensorAccel__Sequence *)malloc(sizeof(px4_msgs__msg__SensorAccel__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorAccel__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorAccel__Sequence__destroy(px4_msgs__msg__SensorAccel__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorAccel__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorAccel__Sequence__are_equal(const px4_msgs__msg__SensorAccel__Sequence * lhs, const px4_msgs__msg__SensorAccel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorAccel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorAccel__Sequence__copy(
  const px4_msgs__msg__SensorAccel__Sequence * input,
  px4_msgs__msg__SensorAccel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorAccel);
    px4_msgs__msg__SensorAccel * data =
      (px4_msgs__msg__SensorAccel *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorAccel__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorAccel__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorAccel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
