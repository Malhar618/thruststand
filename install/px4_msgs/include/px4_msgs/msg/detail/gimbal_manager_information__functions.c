// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/GimbalManagerInformation.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/gimbal_manager_information__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__GimbalManagerInformation__init(px4_msgs__msg__GimbalManagerInformation * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // cap_flags
  // gimbal_device_id
  // roll_min
  // roll_max
  // pitch_min
  // pitch_max
  // yaw_min
  // yaw_max
  return true;
}

void
px4_msgs__msg__GimbalManagerInformation__fini(px4_msgs__msg__GimbalManagerInformation * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // cap_flags
  // gimbal_device_id
  // roll_min
  // roll_max
  // pitch_min
  // pitch_max
  // yaw_min
  // yaw_max
}

bool
px4_msgs__msg__GimbalManagerInformation__are_equal(const px4_msgs__msg__GimbalManagerInformation * lhs, const px4_msgs__msg__GimbalManagerInformation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // cap_flags
  if (lhs->cap_flags != rhs->cap_flags) {
    return false;
  }
  // gimbal_device_id
  if (lhs->gimbal_device_id != rhs->gimbal_device_id) {
    return false;
  }
  // roll_min
  if (lhs->roll_min != rhs->roll_min) {
    return false;
  }
  // roll_max
  if (lhs->roll_max != rhs->roll_max) {
    return false;
  }
  // pitch_min
  if (lhs->pitch_min != rhs->pitch_min) {
    return false;
  }
  // pitch_max
  if (lhs->pitch_max != rhs->pitch_max) {
    return false;
  }
  // yaw_min
  if (lhs->yaw_min != rhs->yaw_min) {
    return false;
  }
  // yaw_max
  if (lhs->yaw_max != rhs->yaw_max) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__GimbalManagerInformation__copy(
  const px4_msgs__msg__GimbalManagerInformation * input,
  px4_msgs__msg__GimbalManagerInformation * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // cap_flags
  output->cap_flags = input->cap_flags;
  // gimbal_device_id
  output->gimbal_device_id = input->gimbal_device_id;
  // roll_min
  output->roll_min = input->roll_min;
  // roll_max
  output->roll_max = input->roll_max;
  // pitch_min
  output->pitch_min = input->pitch_min;
  // pitch_max
  output->pitch_max = input->pitch_max;
  // yaw_min
  output->yaw_min = input->yaw_min;
  // yaw_max
  output->yaw_max = input->yaw_max;
  return true;
}

px4_msgs__msg__GimbalManagerInformation *
px4_msgs__msg__GimbalManagerInformation__create()
{
  px4_msgs__msg__GimbalManagerInformation * msg = (px4_msgs__msg__GimbalManagerInformation *)malloc(sizeof(px4_msgs__msg__GimbalManagerInformation));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__GimbalManagerInformation));
  bool success = px4_msgs__msg__GimbalManagerInformation__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__GimbalManagerInformation__destroy(px4_msgs__msg__GimbalManagerInformation * msg)
{
  if (msg) {
    px4_msgs__msg__GimbalManagerInformation__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__GimbalManagerInformation__Sequence__init(px4_msgs__msg__GimbalManagerInformation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__GimbalManagerInformation * data = NULL;
  if (size) {
    data = (px4_msgs__msg__GimbalManagerInformation *)calloc(size, sizeof(px4_msgs__msg__GimbalManagerInformation));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__GimbalManagerInformation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__GimbalManagerInformation__fini(&data[i - 1]);
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
px4_msgs__msg__GimbalManagerInformation__Sequence__fini(px4_msgs__msg__GimbalManagerInformation__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__GimbalManagerInformation__fini(&array->data[i]);
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

px4_msgs__msg__GimbalManagerInformation__Sequence *
px4_msgs__msg__GimbalManagerInformation__Sequence__create(size_t size)
{
  px4_msgs__msg__GimbalManagerInformation__Sequence * array = (px4_msgs__msg__GimbalManagerInformation__Sequence *)malloc(sizeof(px4_msgs__msg__GimbalManagerInformation__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__GimbalManagerInformation__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__GimbalManagerInformation__Sequence__destroy(px4_msgs__msg__GimbalManagerInformation__Sequence * array)
{
  if (array) {
    px4_msgs__msg__GimbalManagerInformation__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__GimbalManagerInformation__Sequence__are_equal(const px4_msgs__msg__GimbalManagerInformation__Sequence * lhs, const px4_msgs__msg__GimbalManagerInformation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__GimbalManagerInformation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__GimbalManagerInformation__Sequence__copy(
  const px4_msgs__msg__GimbalManagerInformation__Sequence * input,
  px4_msgs__msg__GimbalManagerInformation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__GimbalManagerInformation);
    px4_msgs__msg__GimbalManagerInformation * data =
      (px4_msgs__msg__GimbalManagerInformation *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__GimbalManagerInformation__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__GimbalManagerInformation__fini(&data[i]);
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
    if (!px4_msgs__msg__GimbalManagerInformation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
