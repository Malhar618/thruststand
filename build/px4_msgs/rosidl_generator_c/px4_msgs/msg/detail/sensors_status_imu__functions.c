// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorsStatusImu.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensors_status_imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorsStatusImu__init(px4_msgs__msg__SensorsStatusImu * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // accel_device_id_primary
  // accel_device_ids
  // accel_inconsistency_m_s_s
  // accel_healthy
  // accel_priority
  // gyro_device_id_primary
  // gyro_device_ids
  // gyro_inconsistency_rad_s
  // gyro_healthy
  // gyro_priority
  return true;
}

void
px4_msgs__msg__SensorsStatusImu__fini(px4_msgs__msg__SensorsStatusImu * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // accel_device_id_primary
  // accel_device_ids
  // accel_inconsistency_m_s_s
  // accel_healthy
  // accel_priority
  // gyro_device_id_primary
  // gyro_device_ids
  // gyro_inconsistency_rad_s
  // gyro_healthy
  // gyro_priority
}

bool
px4_msgs__msg__SensorsStatusImu__are_equal(const px4_msgs__msg__SensorsStatusImu * lhs, const px4_msgs__msg__SensorsStatusImu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // accel_device_id_primary
  if (lhs->accel_device_id_primary != rhs->accel_device_id_primary) {
    return false;
  }
  // accel_device_ids
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->accel_device_ids[i] != rhs->accel_device_ids[i]) {
      return false;
    }
  }
  // accel_inconsistency_m_s_s
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->accel_inconsistency_m_s_s[i] != rhs->accel_inconsistency_m_s_s[i]) {
      return false;
    }
  }
  // accel_healthy
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->accel_healthy[i] != rhs->accel_healthy[i]) {
      return false;
    }
  }
  // accel_priority
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->accel_priority[i] != rhs->accel_priority[i]) {
      return false;
    }
  }
  // gyro_device_id_primary
  if (lhs->gyro_device_id_primary != rhs->gyro_device_id_primary) {
    return false;
  }
  // gyro_device_ids
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->gyro_device_ids[i] != rhs->gyro_device_ids[i]) {
      return false;
    }
  }
  // gyro_inconsistency_rad_s
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->gyro_inconsistency_rad_s[i] != rhs->gyro_inconsistency_rad_s[i]) {
      return false;
    }
  }
  // gyro_healthy
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->gyro_healthy[i] != rhs->gyro_healthy[i]) {
      return false;
    }
  }
  // gyro_priority
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->gyro_priority[i] != rhs->gyro_priority[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorsStatusImu__copy(
  const px4_msgs__msg__SensorsStatusImu * input,
  px4_msgs__msg__SensorsStatusImu * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // accel_device_id_primary
  output->accel_device_id_primary = input->accel_device_id_primary;
  // accel_device_ids
  for (size_t i = 0; i < 4; ++i) {
    output->accel_device_ids[i] = input->accel_device_ids[i];
  }
  // accel_inconsistency_m_s_s
  for (size_t i = 0; i < 4; ++i) {
    output->accel_inconsistency_m_s_s[i] = input->accel_inconsistency_m_s_s[i];
  }
  // accel_healthy
  for (size_t i = 0; i < 4; ++i) {
    output->accel_healthy[i] = input->accel_healthy[i];
  }
  // accel_priority
  for (size_t i = 0; i < 4; ++i) {
    output->accel_priority[i] = input->accel_priority[i];
  }
  // gyro_device_id_primary
  output->gyro_device_id_primary = input->gyro_device_id_primary;
  // gyro_device_ids
  for (size_t i = 0; i < 4; ++i) {
    output->gyro_device_ids[i] = input->gyro_device_ids[i];
  }
  // gyro_inconsistency_rad_s
  for (size_t i = 0; i < 4; ++i) {
    output->gyro_inconsistency_rad_s[i] = input->gyro_inconsistency_rad_s[i];
  }
  // gyro_healthy
  for (size_t i = 0; i < 4; ++i) {
    output->gyro_healthy[i] = input->gyro_healthy[i];
  }
  // gyro_priority
  for (size_t i = 0; i < 4; ++i) {
    output->gyro_priority[i] = input->gyro_priority[i];
  }
  return true;
}

px4_msgs__msg__SensorsStatusImu *
px4_msgs__msg__SensorsStatusImu__create()
{
  px4_msgs__msg__SensorsStatusImu * msg = (px4_msgs__msg__SensorsStatusImu *)malloc(sizeof(px4_msgs__msg__SensorsStatusImu));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorsStatusImu));
  bool success = px4_msgs__msg__SensorsStatusImu__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorsStatusImu__destroy(px4_msgs__msg__SensorsStatusImu * msg)
{
  if (msg) {
    px4_msgs__msg__SensorsStatusImu__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorsStatusImu__Sequence__init(px4_msgs__msg__SensorsStatusImu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorsStatusImu * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorsStatusImu *)calloc(size, sizeof(px4_msgs__msg__SensorsStatusImu));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorsStatusImu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorsStatusImu__fini(&data[i - 1]);
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
px4_msgs__msg__SensorsStatusImu__Sequence__fini(px4_msgs__msg__SensorsStatusImu__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorsStatusImu__fini(&array->data[i]);
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

px4_msgs__msg__SensorsStatusImu__Sequence *
px4_msgs__msg__SensorsStatusImu__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorsStatusImu__Sequence * array = (px4_msgs__msg__SensorsStatusImu__Sequence *)malloc(sizeof(px4_msgs__msg__SensorsStatusImu__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorsStatusImu__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorsStatusImu__Sequence__destroy(px4_msgs__msg__SensorsStatusImu__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorsStatusImu__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorsStatusImu__Sequence__are_equal(const px4_msgs__msg__SensorsStatusImu__Sequence * lhs, const px4_msgs__msg__SensorsStatusImu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorsStatusImu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorsStatusImu__Sequence__copy(
  const px4_msgs__msg__SensorsStatusImu__Sequence * input,
  px4_msgs__msg__SensorsStatusImu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorsStatusImu);
    px4_msgs__msg__SensorsStatusImu * data =
      (px4_msgs__msg__SensorsStatusImu *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorsStatusImu__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorsStatusImu__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorsStatusImu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
