// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorGyroFft.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_gyro_fft__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorGyroFft__init(px4_msgs__msg__SensorGyroFft * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // sensor_sample_rate_hz
  // resolution_hz
  // peak_frequencies_x
  // peak_frequencies_y
  // peak_frequencies_z
  // peak_snr_x
  // peak_snr_y
  // peak_snr_z
  return true;
}

void
px4_msgs__msg__SensorGyroFft__fini(px4_msgs__msg__SensorGyroFft * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // sensor_sample_rate_hz
  // resolution_hz
  // peak_frequencies_x
  // peak_frequencies_y
  // peak_frequencies_z
  // peak_snr_x
  // peak_snr_y
  // peak_snr_z
}

bool
px4_msgs__msg__SensorGyroFft__are_equal(const px4_msgs__msg__SensorGyroFft * lhs, const px4_msgs__msg__SensorGyroFft * rhs)
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
  // sensor_sample_rate_hz
  if (lhs->sensor_sample_rate_hz != rhs->sensor_sample_rate_hz) {
    return false;
  }
  // resolution_hz
  if (lhs->resolution_hz != rhs->resolution_hz) {
    return false;
  }
  // peak_frequencies_x
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_frequencies_x[i] != rhs->peak_frequencies_x[i]) {
      return false;
    }
  }
  // peak_frequencies_y
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_frequencies_y[i] != rhs->peak_frequencies_y[i]) {
      return false;
    }
  }
  // peak_frequencies_z
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_frequencies_z[i] != rhs->peak_frequencies_z[i]) {
      return false;
    }
  }
  // peak_snr_x
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_snr_x[i] != rhs->peak_snr_x[i]) {
      return false;
    }
  }
  // peak_snr_y
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_snr_y[i] != rhs->peak_snr_y[i]) {
      return false;
    }
  }
  // peak_snr_z
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->peak_snr_z[i] != rhs->peak_snr_z[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorGyroFft__copy(
  const px4_msgs__msg__SensorGyroFft * input,
  px4_msgs__msg__SensorGyroFft * output)
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
  // sensor_sample_rate_hz
  output->sensor_sample_rate_hz = input->sensor_sample_rate_hz;
  // resolution_hz
  output->resolution_hz = input->resolution_hz;
  // peak_frequencies_x
  for (size_t i = 0; i < 3; ++i) {
    output->peak_frequencies_x[i] = input->peak_frequencies_x[i];
  }
  // peak_frequencies_y
  for (size_t i = 0; i < 3; ++i) {
    output->peak_frequencies_y[i] = input->peak_frequencies_y[i];
  }
  // peak_frequencies_z
  for (size_t i = 0; i < 3; ++i) {
    output->peak_frequencies_z[i] = input->peak_frequencies_z[i];
  }
  // peak_snr_x
  for (size_t i = 0; i < 3; ++i) {
    output->peak_snr_x[i] = input->peak_snr_x[i];
  }
  // peak_snr_y
  for (size_t i = 0; i < 3; ++i) {
    output->peak_snr_y[i] = input->peak_snr_y[i];
  }
  // peak_snr_z
  for (size_t i = 0; i < 3; ++i) {
    output->peak_snr_z[i] = input->peak_snr_z[i];
  }
  return true;
}

px4_msgs__msg__SensorGyroFft *
px4_msgs__msg__SensorGyroFft__create()
{
  px4_msgs__msg__SensorGyroFft * msg = (px4_msgs__msg__SensorGyroFft *)malloc(sizeof(px4_msgs__msg__SensorGyroFft));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorGyroFft));
  bool success = px4_msgs__msg__SensorGyroFft__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorGyroFft__destroy(px4_msgs__msg__SensorGyroFft * msg)
{
  if (msg) {
    px4_msgs__msg__SensorGyroFft__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorGyroFft__Sequence__init(px4_msgs__msg__SensorGyroFft__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorGyroFft * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorGyroFft *)calloc(size, sizeof(px4_msgs__msg__SensorGyroFft));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorGyroFft__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorGyroFft__fini(&data[i - 1]);
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
px4_msgs__msg__SensorGyroFft__Sequence__fini(px4_msgs__msg__SensorGyroFft__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorGyroFft__fini(&array->data[i]);
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

px4_msgs__msg__SensorGyroFft__Sequence *
px4_msgs__msg__SensorGyroFft__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorGyroFft__Sequence * array = (px4_msgs__msg__SensorGyroFft__Sequence *)malloc(sizeof(px4_msgs__msg__SensorGyroFft__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorGyroFft__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorGyroFft__Sequence__destroy(px4_msgs__msg__SensorGyroFft__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorGyroFft__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorGyroFft__Sequence__are_equal(const px4_msgs__msg__SensorGyroFft__Sequence * lhs, const px4_msgs__msg__SensorGyroFft__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorGyroFft__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorGyroFft__Sequence__copy(
  const px4_msgs__msg__SensorGyroFft__Sequence * input,
  px4_msgs__msg__SensorGyroFft__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorGyroFft);
    px4_msgs__msg__SensorGyroFft * data =
      (px4_msgs__msg__SensorGyroFft *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorGyroFft__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorGyroFft__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorGyroFft__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
