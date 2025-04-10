// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/ActuatorMotors.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/actuator_motors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__ActuatorMotors__init(px4_msgs__msg__ActuatorMotors * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // reversible_flags
  // control
  return true;
}

void
px4_msgs__msg__ActuatorMotors__fini(px4_msgs__msg__ActuatorMotors * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // reversible_flags
  // control
}

bool
px4_msgs__msg__ActuatorMotors__are_equal(const px4_msgs__msg__ActuatorMotors * lhs, const px4_msgs__msg__ActuatorMotors * rhs)
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
  // reversible_flags
  if (lhs->reversible_flags != rhs->reversible_flags) {
    return false;
  }
  // control
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->control[i] != rhs->control[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ActuatorMotors__copy(
  const px4_msgs__msg__ActuatorMotors * input,
  px4_msgs__msg__ActuatorMotors * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // reversible_flags
  output->reversible_flags = input->reversible_flags;
  // control
  for (size_t i = 0; i < 12; ++i) {
    output->control[i] = input->control[i];
  }
  return true;
}

px4_msgs__msg__ActuatorMotors *
px4_msgs__msg__ActuatorMotors__create()
{
  px4_msgs__msg__ActuatorMotors * msg = (px4_msgs__msg__ActuatorMotors *)malloc(sizeof(px4_msgs__msg__ActuatorMotors));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__ActuatorMotors));
  bool success = px4_msgs__msg__ActuatorMotors__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__ActuatorMotors__destroy(px4_msgs__msg__ActuatorMotors * msg)
{
  if (msg) {
    px4_msgs__msg__ActuatorMotors__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__ActuatorMotors__Sequence__init(px4_msgs__msg__ActuatorMotors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__ActuatorMotors * data = NULL;
  if (size) {
    data = (px4_msgs__msg__ActuatorMotors *)calloc(size, sizeof(px4_msgs__msg__ActuatorMotors));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__ActuatorMotors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__ActuatorMotors__fini(&data[i - 1]);
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
px4_msgs__msg__ActuatorMotors__Sequence__fini(px4_msgs__msg__ActuatorMotors__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__ActuatorMotors__fini(&array->data[i]);
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

px4_msgs__msg__ActuatorMotors__Sequence *
px4_msgs__msg__ActuatorMotors__Sequence__create(size_t size)
{
  px4_msgs__msg__ActuatorMotors__Sequence * array = (px4_msgs__msg__ActuatorMotors__Sequence *)malloc(sizeof(px4_msgs__msg__ActuatorMotors__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__ActuatorMotors__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__ActuatorMotors__Sequence__destroy(px4_msgs__msg__ActuatorMotors__Sequence * array)
{
  if (array) {
    px4_msgs__msg__ActuatorMotors__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__ActuatorMotors__Sequence__are_equal(const px4_msgs__msg__ActuatorMotors__Sequence * lhs, const px4_msgs__msg__ActuatorMotors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__ActuatorMotors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ActuatorMotors__Sequence__copy(
  const px4_msgs__msg__ActuatorMotors__Sequence * input,
  px4_msgs__msg__ActuatorMotors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__ActuatorMotors);
    px4_msgs__msg__ActuatorMotors * data =
      (px4_msgs__msg__ActuatorMotors *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__ActuatorMotors__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__ActuatorMotors__fini(&data[i]);
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
    if (!px4_msgs__msg__ActuatorMotors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
