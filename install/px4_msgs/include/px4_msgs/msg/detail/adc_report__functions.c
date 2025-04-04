// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/AdcReport.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/adc_report__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__AdcReport__init(px4_msgs__msg__AdcReport * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // device_id
  // channel_id
  // raw_data
  // resolution
  // v_ref
  return true;
}

void
px4_msgs__msg__AdcReport__fini(px4_msgs__msg__AdcReport * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // device_id
  // channel_id
  // raw_data
  // resolution
  // v_ref
}

bool
px4_msgs__msg__AdcReport__are_equal(const px4_msgs__msg__AdcReport * lhs, const px4_msgs__msg__AdcReport * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // device_id
  if (lhs->device_id != rhs->device_id) {
    return false;
  }
  // channel_id
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->channel_id[i] != rhs->channel_id[i]) {
      return false;
    }
  }
  // raw_data
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->raw_data[i] != rhs->raw_data[i]) {
      return false;
    }
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // v_ref
  if (lhs->v_ref != rhs->v_ref) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__AdcReport__copy(
  const px4_msgs__msg__AdcReport * input,
  px4_msgs__msg__AdcReport * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // device_id
  output->device_id = input->device_id;
  // channel_id
  for (size_t i = 0; i < 12; ++i) {
    output->channel_id[i] = input->channel_id[i];
  }
  // raw_data
  for (size_t i = 0; i < 12; ++i) {
    output->raw_data[i] = input->raw_data[i];
  }
  // resolution
  output->resolution = input->resolution;
  // v_ref
  output->v_ref = input->v_ref;
  return true;
}

px4_msgs__msg__AdcReport *
px4_msgs__msg__AdcReport__create()
{
  px4_msgs__msg__AdcReport * msg = (px4_msgs__msg__AdcReport *)malloc(sizeof(px4_msgs__msg__AdcReport));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__AdcReport));
  bool success = px4_msgs__msg__AdcReport__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__AdcReport__destroy(px4_msgs__msg__AdcReport * msg)
{
  if (msg) {
    px4_msgs__msg__AdcReport__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__AdcReport__Sequence__init(px4_msgs__msg__AdcReport__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__AdcReport * data = NULL;
  if (size) {
    data = (px4_msgs__msg__AdcReport *)calloc(size, sizeof(px4_msgs__msg__AdcReport));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__AdcReport__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__AdcReport__fini(&data[i - 1]);
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
px4_msgs__msg__AdcReport__Sequence__fini(px4_msgs__msg__AdcReport__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__AdcReport__fini(&array->data[i]);
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

px4_msgs__msg__AdcReport__Sequence *
px4_msgs__msg__AdcReport__Sequence__create(size_t size)
{
  px4_msgs__msg__AdcReport__Sequence * array = (px4_msgs__msg__AdcReport__Sequence *)malloc(sizeof(px4_msgs__msg__AdcReport__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__AdcReport__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__AdcReport__Sequence__destroy(px4_msgs__msg__AdcReport__Sequence * array)
{
  if (array) {
    px4_msgs__msg__AdcReport__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__AdcReport__Sequence__are_equal(const px4_msgs__msg__AdcReport__Sequence * lhs, const px4_msgs__msg__AdcReport__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__AdcReport__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__AdcReport__Sequence__copy(
  const px4_msgs__msg__AdcReport__Sequence * input,
  px4_msgs__msg__AdcReport__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__AdcReport);
    px4_msgs__msg__AdcReport * data =
      (px4_msgs__msg__AdcReport *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__AdcReport__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__AdcReport__fini(&data[i]);
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
    if (!px4_msgs__msg__AdcReport__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
