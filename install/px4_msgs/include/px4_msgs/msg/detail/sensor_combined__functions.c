// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorCombined.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_combined__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorCombined__init(px4_msgs__msg__SensorCombined * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // gyro_rad
  // gyro_integral_dt
  // accelerometer_timestamp_relative
  // accelerometer_m_s2
  // accelerometer_integral_dt
  // accelerometer_clipping
  // gyro_clipping
  // accel_calibration_count
  // gyro_calibration_count
  return true;
}

void
px4_msgs__msg__SensorCombined__fini(px4_msgs__msg__SensorCombined * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // gyro_rad
  // gyro_integral_dt
  // accelerometer_timestamp_relative
  // accelerometer_m_s2
  // accelerometer_integral_dt
  // accelerometer_clipping
  // gyro_clipping
  // accel_calibration_count
  // gyro_calibration_count
}

bool
px4_msgs__msg__SensorCombined__are_equal(const px4_msgs__msg__SensorCombined * lhs, const px4_msgs__msg__SensorCombined * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // gyro_rad
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro_rad[i] != rhs->gyro_rad[i]) {
      return false;
    }
  }
  // gyro_integral_dt
  if (lhs->gyro_integral_dt != rhs->gyro_integral_dt) {
    return false;
  }
  // accelerometer_timestamp_relative
  if (lhs->accelerometer_timestamp_relative != rhs->accelerometer_timestamp_relative) {
    return false;
  }
  // accelerometer_m_s2
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->accelerometer_m_s2[i] != rhs->accelerometer_m_s2[i]) {
      return false;
    }
  }
  // accelerometer_integral_dt
  if (lhs->accelerometer_integral_dt != rhs->accelerometer_integral_dt) {
    return false;
  }
  // accelerometer_clipping
  if (lhs->accelerometer_clipping != rhs->accelerometer_clipping) {
    return false;
  }
  // gyro_clipping
  if (lhs->gyro_clipping != rhs->gyro_clipping) {
    return false;
  }
  // accel_calibration_count
  if (lhs->accel_calibration_count != rhs->accel_calibration_count) {
    return false;
  }
  // gyro_calibration_count
  if (lhs->gyro_calibration_count != rhs->gyro_calibration_count) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__SensorCombined__copy(
  const px4_msgs__msg__SensorCombined * input,
  px4_msgs__msg__SensorCombined * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // gyro_rad
  for (size_t i = 0; i < 3; ++i) {
    output->gyro_rad[i] = input->gyro_rad[i];
  }
  // gyro_integral_dt
  output->gyro_integral_dt = input->gyro_integral_dt;
  // accelerometer_timestamp_relative
  output->accelerometer_timestamp_relative = input->accelerometer_timestamp_relative;
  // accelerometer_m_s2
  for (size_t i = 0; i < 3; ++i) {
    output->accelerometer_m_s2[i] = input->accelerometer_m_s2[i];
  }
  // accelerometer_integral_dt
  output->accelerometer_integral_dt = input->accelerometer_integral_dt;
  // accelerometer_clipping
  output->accelerometer_clipping = input->accelerometer_clipping;
  // gyro_clipping
  output->gyro_clipping = input->gyro_clipping;
  // accel_calibration_count
  output->accel_calibration_count = input->accel_calibration_count;
  // gyro_calibration_count
  output->gyro_calibration_count = input->gyro_calibration_count;
  return true;
}

px4_msgs__msg__SensorCombined *
px4_msgs__msg__SensorCombined__create()
{
  px4_msgs__msg__SensorCombined * msg = (px4_msgs__msg__SensorCombined *)malloc(sizeof(px4_msgs__msg__SensorCombined));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorCombined));
  bool success = px4_msgs__msg__SensorCombined__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorCombined__destroy(px4_msgs__msg__SensorCombined * msg)
{
  if (msg) {
    px4_msgs__msg__SensorCombined__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorCombined__Sequence__init(px4_msgs__msg__SensorCombined__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorCombined * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorCombined *)calloc(size, sizeof(px4_msgs__msg__SensorCombined));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorCombined__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorCombined__fini(&data[i - 1]);
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
px4_msgs__msg__SensorCombined__Sequence__fini(px4_msgs__msg__SensorCombined__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorCombined__fini(&array->data[i]);
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

px4_msgs__msg__SensorCombined__Sequence *
px4_msgs__msg__SensorCombined__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorCombined__Sequence * array = (px4_msgs__msg__SensorCombined__Sequence *)malloc(sizeof(px4_msgs__msg__SensorCombined__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorCombined__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorCombined__Sequence__destroy(px4_msgs__msg__SensorCombined__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorCombined__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorCombined__Sequence__are_equal(const px4_msgs__msg__SensorCombined__Sequence * lhs, const px4_msgs__msg__SensorCombined__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorCombined__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorCombined__Sequence__copy(
  const px4_msgs__msg__SensorCombined__Sequence * input,
  px4_msgs__msg__SensorCombined__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorCombined);
    px4_msgs__msg__SensorCombined * data =
      (px4_msgs__msg__SensorCombined *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorCombined__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorCombined__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorCombined__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
