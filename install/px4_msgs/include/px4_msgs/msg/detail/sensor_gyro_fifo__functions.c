// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorGyroFifo.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_gyro_fifo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorGyroFifo__init(px4_msgs__msg__SensorGyroFifo * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // dt
  // scale
  // samples
  // x
  // y
  // z
  return true;
}

void
px4_msgs__msg__SensorGyroFifo__fini(px4_msgs__msg__SensorGyroFifo * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // dt
  // scale
  // samples
  // x
  // y
  // z
}

bool
px4_msgs__msg__SensorGyroFifo__are_equal(const px4_msgs__msg__SensorGyroFifo * lhs, const px4_msgs__msg__SensorGyroFifo * rhs)
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
  // dt
  if (lhs->dt != rhs->dt) {
    return false;
  }
  // scale
  if (lhs->scale != rhs->scale) {
    return false;
  }
  // samples
  if (lhs->samples != rhs->samples) {
    return false;
  }
  // x
  for (size_t i = 0; i < 32; ++i) {
    if (lhs->x[i] != rhs->x[i]) {
      return false;
    }
  }
  // y
  for (size_t i = 0; i < 32; ++i) {
    if (lhs->y[i] != rhs->y[i]) {
      return false;
    }
  }
  // z
  for (size_t i = 0; i < 32; ++i) {
    if (lhs->z[i] != rhs->z[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorGyroFifo__copy(
  const px4_msgs__msg__SensorGyroFifo * input,
  px4_msgs__msg__SensorGyroFifo * output)
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
  // dt
  output->dt = input->dt;
  // scale
  output->scale = input->scale;
  // samples
  output->samples = input->samples;
  // x
  for (size_t i = 0; i < 32; ++i) {
    output->x[i] = input->x[i];
  }
  // y
  for (size_t i = 0; i < 32; ++i) {
    output->y[i] = input->y[i];
  }
  // z
  for (size_t i = 0; i < 32; ++i) {
    output->z[i] = input->z[i];
  }
  return true;
}

px4_msgs__msg__SensorGyroFifo *
px4_msgs__msg__SensorGyroFifo__create()
{
  px4_msgs__msg__SensorGyroFifo * msg = (px4_msgs__msg__SensorGyroFifo *)malloc(sizeof(px4_msgs__msg__SensorGyroFifo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorGyroFifo));
  bool success = px4_msgs__msg__SensorGyroFifo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorGyroFifo__destroy(px4_msgs__msg__SensorGyroFifo * msg)
{
  if (msg) {
    px4_msgs__msg__SensorGyroFifo__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorGyroFifo__Sequence__init(px4_msgs__msg__SensorGyroFifo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorGyroFifo * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorGyroFifo *)calloc(size, sizeof(px4_msgs__msg__SensorGyroFifo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorGyroFifo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorGyroFifo__fini(&data[i - 1]);
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
px4_msgs__msg__SensorGyroFifo__Sequence__fini(px4_msgs__msg__SensorGyroFifo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorGyroFifo__fini(&array->data[i]);
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

px4_msgs__msg__SensorGyroFifo__Sequence *
px4_msgs__msg__SensorGyroFifo__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorGyroFifo__Sequence * array = (px4_msgs__msg__SensorGyroFifo__Sequence *)malloc(sizeof(px4_msgs__msg__SensorGyroFifo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorGyroFifo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorGyroFifo__Sequence__destroy(px4_msgs__msg__SensorGyroFifo__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorGyroFifo__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorGyroFifo__Sequence__are_equal(const px4_msgs__msg__SensorGyroFifo__Sequence * lhs, const px4_msgs__msg__SensorGyroFifo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorGyroFifo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorGyroFifo__Sequence__copy(
  const px4_msgs__msg__SensorGyroFifo__Sequence * input,
  px4_msgs__msg__SensorGyroFifo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorGyroFifo);
    px4_msgs__msg__SensorGyroFifo * data =
      (px4_msgs__msg__SensorGyroFifo *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorGyroFifo__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorGyroFifo__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorGyroFifo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
