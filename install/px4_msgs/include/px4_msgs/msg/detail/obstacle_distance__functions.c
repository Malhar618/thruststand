// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/ObstacleDistance.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/obstacle_distance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__ObstacleDistance__init(px4_msgs__msg__ObstacleDistance * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // frame
  // sensor_type
  // distances
  // increment
  // min_distance
  // max_distance
  // angle_offset
  return true;
}

void
px4_msgs__msg__ObstacleDistance__fini(px4_msgs__msg__ObstacleDistance * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // frame
  // sensor_type
  // distances
  // increment
  // min_distance
  // max_distance
  // angle_offset
}

bool
px4_msgs__msg__ObstacleDistance__are_equal(const px4_msgs__msg__ObstacleDistance * lhs, const px4_msgs__msg__ObstacleDistance * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // frame
  if (lhs->frame != rhs->frame) {
    return false;
  }
  // sensor_type
  if (lhs->sensor_type != rhs->sensor_type) {
    return false;
  }
  // distances
  for (size_t i = 0; i < 72; ++i) {
    if (lhs->distances[i] != rhs->distances[i]) {
      return false;
    }
  }
  // increment
  if (lhs->increment != rhs->increment) {
    return false;
  }
  // min_distance
  if (lhs->min_distance != rhs->min_distance) {
    return false;
  }
  // max_distance
  if (lhs->max_distance != rhs->max_distance) {
    return false;
  }
  // angle_offset
  if (lhs->angle_offset != rhs->angle_offset) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__ObstacleDistance__copy(
  const px4_msgs__msg__ObstacleDistance * input,
  px4_msgs__msg__ObstacleDistance * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // frame
  output->frame = input->frame;
  // sensor_type
  output->sensor_type = input->sensor_type;
  // distances
  for (size_t i = 0; i < 72; ++i) {
    output->distances[i] = input->distances[i];
  }
  // increment
  output->increment = input->increment;
  // min_distance
  output->min_distance = input->min_distance;
  // max_distance
  output->max_distance = input->max_distance;
  // angle_offset
  output->angle_offset = input->angle_offset;
  return true;
}

px4_msgs__msg__ObstacleDistance *
px4_msgs__msg__ObstacleDistance__create()
{
  px4_msgs__msg__ObstacleDistance * msg = (px4_msgs__msg__ObstacleDistance *)malloc(sizeof(px4_msgs__msg__ObstacleDistance));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__ObstacleDistance));
  bool success = px4_msgs__msg__ObstacleDistance__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__ObstacleDistance__destroy(px4_msgs__msg__ObstacleDistance * msg)
{
  if (msg) {
    px4_msgs__msg__ObstacleDistance__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__ObstacleDistance__Sequence__init(px4_msgs__msg__ObstacleDistance__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__ObstacleDistance * data = NULL;
  if (size) {
    data = (px4_msgs__msg__ObstacleDistance *)calloc(size, sizeof(px4_msgs__msg__ObstacleDistance));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__ObstacleDistance__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__ObstacleDistance__fini(&data[i - 1]);
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
px4_msgs__msg__ObstacleDistance__Sequence__fini(px4_msgs__msg__ObstacleDistance__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__ObstacleDistance__fini(&array->data[i]);
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

px4_msgs__msg__ObstacleDistance__Sequence *
px4_msgs__msg__ObstacleDistance__Sequence__create(size_t size)
{
  px4_msgs__msg__ObstacleDistance__Sequence * array = (px4_msgs__msg__ObstacleDistance__Sequence *)malloc(sizeof(px4_msgs__msg__ObstacleDistance__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__ObstacleDistance__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__ObstacleDistance__Sequence__destroy(px4_msgs__msg__ObstacleDistance__Sequence * array)
{
  if (array) {
    px4_msgs__msg__ObstacleDistance__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__ObstacleDistance__Sequence__are_equal(const px4_msgs__msg__ObstacleDistance__Sequence * lhs, const px4_msgs__msg__ObstacleDistance__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__ObstacleDistance__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ObstacleDistance__Sequence__copy(
  const px4_msgs__msg__ObstacleDistance__Sequence * input,
  px4_msgs__msg__ObstacleDistance__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__ObstacleDistance);
    px4_msgs__msg__ObstacleDistance * data =
      (px4_msgs__msg__ObstacleDistance *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__ObstacleDistance__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__ObstacleDistance__fini(&data[i]);
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
    if (!px4_msgs__msg__ObstacleDistance__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
