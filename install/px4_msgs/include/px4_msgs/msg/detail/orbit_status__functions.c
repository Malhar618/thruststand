// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/OrbitStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/orbit_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__OrbitStatus__init(px4_msgs__msg__OrbitStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // radius
  // frame
  // x
  // y
  // z
  // yaw_behaviour
  return true;
}

void
px4_msgs__msg__OrbitStatus__fini(px4_msgs__msg__OrbitStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // radius
  // frame
  // x
  // y
  // z
  // yaw_behaviour
}

bool
px4_msgs__msg__OrbitStatus__are_equal(const px4_msgs__msg__OrbitStatus * lhs, const px4_msgs__msg__OrbitStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  // frame
  if (lhs->frame != rhs->frame) {
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
  // yaw_behaviour
  if (lhs->yaw_behaviour != rhs->yaw_behaviour) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__OrbitStatus__copy(
  const px4_msgs__msg__OrbitStatus * input,
  px4_msgs__msg__OrbitStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // radius
  output->radius = input->radius;
  // frame
  output->frame = input->frame;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // yaw_behaviour
  output->yaw_behaviour = input->yaw_behaviour;
  return true;
}

px4_msgs__msg__OrbitStatus *
px4_msgs__msg__OrbitStatus__create()
{
  px4_msgs__msg__OrbitStatus * msg = (px4_msgs__msg__OrbitStatus *)malloc(sizeof(px4_msgs__msg__OrbitStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__OrbitStatus));
  bool success = px4_msgs__msg__OrbitStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__OrbitStatus__destroy(px4_msgs__msg__OrbitStatus * msg)
{
  if (msg) {
    px4_msgs__msg__OrbitStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__OrbitStatus__Sequence__init(px4_msgs__msg__OrbitStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__OrbitStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__OrbitStatus *)calloc(size, sizeof(px4_msgs__msg__OrbitStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__OrbitStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__OrbitStatus__fini(&data[i - 1]);
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
px4_msgs__msg__OrbitStatus__Sequence__fini(px4_msgs__msg__OrbitStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__OrbitStatus__fini(&array->data[i]);
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

px4_msgs__msg__OrbitStatus__Sequence *
px4_msgs__msg__OrbitStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__OrbitStatus__Sequence * array = (px4_msgs__msg__OrbitStatus__Sequence *)malloc(sizeof(px4_msgs__msg__OrbitStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__OrbitStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__OrbitStatus__Sequence__destroy(px4_msgs__msg__OrbitStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__OrbitStatus__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__OrbitStatus__Sequence__are_equal(const px4_msgs__msg__OrbitStatus__Sequence * lhs, const px4_msgs__msg__OrbitStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__OrbitStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__OrbitStatus__Sequence__copy(
  const px4_msgs__msg__OrbitStatus__Sequence * input,
  px4_msgs__msg__OrbitStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__OrbitStatus);
    px4_msgs__msg__OrbitStatus * data =
      (px4_msgs__msg__OrbitStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__OrbitStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__OrbitStatus__fini(&data[i]);
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
    if (!px4_msgs__msg__OrbitStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
