// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/LedControl.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/led_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__LedControl__init(px4_msgs__msg__LedControl * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // led_mask
  // color
  // mode
  // num_blinks
  // priority
  return true;
}

void
px4_msgs__msg__LedControl__fini(px4_msgs__msg__LedControl * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // led_mask
  // color
  // mode
  // num_blinks
  // priority
}

bool
px4_msgs__msg__LedControl__are_equal(const px4_msgs__msg__LedControl * lhs, const px4_msgs__msg__LedControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // led_mask
  if (lhs->led_mask != rhs->led_mask) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // num_blinks
  if (lhs->num_blinks != rhs->num_blinks) {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__LedControl__copy(
  const px4_msgs__msg__LedControl * input,
  px4_msgs__msg__LedControl * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // led_mask
  output->led_mask = input->led_mask;
  // color
  output->color = input->color;
  // mode
  output->mode = input->mode;
  // num_blinks
  output->num_blinks = input->num_blinks;
  // priority
  output->priority = input->priority;
  return true;
}

px4_msgs__msg__LedControl *
px4_msgs__msg__LedControl__create()
{
  px4_msgs__msg__LedControl * msg = (px4_msgs__msg__LedControl *)malloc(sizeof(px4_msgs__msg__LedControl));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__LedControl));
  bool success = px4_msgs__msg__LedControl__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__LedControl__destroy(px4_msgs__msg__LedControl * msg)
{
  if (msg) {
    px4_msgs__msg__LedControl__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__LedControl__Sequence__init(px4_msgs__msg__LedControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__LedControl * data = NULL;
  if (size) {
    data = (px4_msgs__msg__LedControl *)calloc(size, sizeof(px4_msgs__msg__LedControl));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__LedControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__LedControl__fini(&data[i - 1]);
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
px4_msgs__msg__LedControl__Sequence__fini(px4_msgs__msg__LedControl__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__LedControl__fini(&array->data[i]);
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

px4_msgs__msg__LedControl__Sequence *
px4_msgs__msg__LedControl__Sequence__create(size_t size)
{
  px4_msgs__msg__LedControl__Sequence * array = (px4_msgs__msg__LedControl__Sequence *)malloc(sizeof(px4_msgs__msg__LedControl__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__LedControl__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__LedControl__Sequence__destroy(px4_msgs__msg__LedControl__Sequence * array)
{
  if (array) {
    px4_msgs__msg__LedControl__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__LedControl__Sequence__are_equal(const px4_msgs__msg__LedControl__Sequence * lhs, const px4_msgs__msg__LedControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__LedControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__LedControl__Sequence__copy(
  const px4_msgs__msg__LedControl__Sequence * input,
  px4_msgs__msg__LedControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__LedControl);
    px4_msgs__msg__LedControl * data =
      (px4_msgs__msg__LedControl *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__LedControl__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__LedControl__fini(&data[i]);
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
    if (!px4_msgs__msg__LedControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
