// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorSelection.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_selection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorSelection__init(px4_msgs__msg__SensorSelection * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // accel_device_id
  // gyro_device_id
  return true;
}

void
px4_msgs__msg__SensorSelection__fini(px4_msgs__msg__SensorSelection * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // accel_device_id
  // gyro_device_id
}

bool
px4_msgs__msg__SensorSelection__are_equal(const px4_msgs__msg__SensorSelection * lhs, const px4_msgs__msg__SensorSelection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // accel_device_id
  if (lhs->accel_device_id != rhs->accel_device_id) {
    return false;
  }
  // gyro_device_id
  if (lhs->gyro_device_id != rhs->gyro_device_id) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__SensorSelection__copy(
  const px4_msgs__msg__SensorSelection * input,
  px4_msgs__msg__SensorSelection * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // accel_device_id
  output->accel_device_id = input->accel_device_id;
  // gyro_device_id
  output->gyro_device_id = input->gyro_device_id;
  return true;
}

px4_msgs__msg__SensorSelection *
px4_msgs__msg__SensorSelection__create()
{
  px4_msgs__msg__SensorSelection * msg = (px4_msgs__msg__SensorSelection *)malloc(sizeof(px4_msgs__msg__SensorSelection));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorSelection));
  bool success = px4_msgs__msg__SensorSelection__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorSelection__destroy(px4_msgs__msg__SensorSelection * msg)
{
  if (msg) {
    px4_msgs__msg__SensorSelection__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorSelection__Sequence__init(px4_msgs__msg__SensorSelection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorSelection * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorSelection *)calloc(size, sizeof(px4_msgs__msg__SensorSelection));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorSelection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorSelection__fini(&data[i - 1]);
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
px4_msgs__msg__SensorSelection__Sequence__fini(px4_msgs__msg__SensorSelection__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorSelection__fini(&array->data[i]);
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

px4_msgs__msg__SensorSelection__Sequence *
px4_msgs__msg__SensorSelection__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorSelection__Sequence * array = (px4_msgs__msg__SensorSelection__Sequence *)malloc(sizeof(px4_msgs__msg__SensorSelection__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorSelection__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorSelection__Sequence__destroy(px4_msgs__msg__SensorSelection__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorSelection__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__SensorSelection__Sequence__are_equal(const px4_msgs__msg__SensorSelection__Sequence * lhs, const px4_msgs__msg__SensorSelection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__SensorSelection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__SensorSelection__Sequence__copy(
  const px4_msgs__msg__SensorSelection__Sequence * input,
  px4_msgs__msg__SensorSelection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__SensorSelection);
    px4_msgs__msg__SensorSelection * data =
      (px4_msgs__msg__SensorSelection *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__SensorSelection__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__SensorSelection__fini(&data[i]);
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
    if (!px4_msgs__msg__SensorSelection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
