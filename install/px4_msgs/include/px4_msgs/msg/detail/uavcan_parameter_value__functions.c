// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/UavcanParameterValue.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/uavcan_parameter_value__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__UavcanParameterValue__init(px4_msgs__msg__UavcanParameterValue * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // node_id
  // param_id
  // param_index
  // param_count
  // param_type
  // int_value
  // real_value
  return true;
}

void
px4_msgs__msg__UavcanParameterValue__fini(px4_msgs__msg__UavcanParameterValue * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // node_id
  // param_id
  // param_index
  // param_count
  // param_type
  // int_value
  // real_value
}

bool
px4_msgs__msg__UavcanParameterValue__are_equal(const px4_msgs__msg__UavcanParameterValue * lhs, const px4_msgs__msg__UavcanParameterValue * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // node_id
  if (lhs->node_id != rhs->node_id) {
    return false;
  }
  // param_id
  for (size_t i = 0; i < 17; ++i) {
    if (lhs->param_id[i] != rhs->param_id[i]) {
      return false;
    }
  }
  // param_index
  if (lhs->param_index != rhs->param_index) {
    return false;
  }
  // param_count
  if (lhs->param_count != rhs->param_count) {
    return false;
  }
  // param_type
  if (lhs->param_type != rhs->param_type) {
    return false;
  }
  // int_value
  if (lhs->int_value != rhs->int_value) {
    return false;
  }
  // real_value
  if (lhs->real_value != rhs->real_value) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__UavcanParameterValue__copy(
  const px4_msgs__msg__UavcanParameterValue * input,
  px4_msgs__msg__UavcanParameterValue * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // node_id
  output->node_id = input->node_id;
  // param_id
  for (size_t i = 0; i < 17; ++i) {
    output->param_id[i] = input->param_id[i];
  }
  // param_index
  output->param_index = input->param_index;
  // param_count
  output->param_count = input->param_count;
  // param_type
  output->param_type = input->param_type;
  // int_value
  output->int_value = input->int_value;
  // real_value
  output->real_value = input->real_value;
  return true;
}

px4_msgs__msg__UavcanParameterValue *
px4_msgs__msg__UavcanParameterValue__create()
{
  px4_msgs__msg__UavcanParameterValue * msg = (px4_msgs__msg__UavcanParameterValue *)malloc(sizeof(px4_msgs__msg__UavcanParameterValue));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__UavcanParameterValue));
  bool success = px4_msgs__msg__UavcanParameterValue__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__UavcanParameterValue__destroy(px4_msgs__msg__UavcanParameterValue * msg)
{
  if (msg) {
    px4_msgs__msg__UavcanParameterValue__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__UavcanParameterValue__Sequence__init(px4_msgs__msg__UavcanParameterValue__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__UavcanParameterValue * data = NULL;
  if (size) {
    data = (px4_msgs__msg__UavcanParameterValue *)calloc(size, sizeof(px4_msgs__msg__UavcanParameterValue));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__UavcanParameterValue__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__UavcanParameterValue__fini(&data[i - 1]);
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
px4_msgs__msg__UavcanParameterValue__Sequence__fini(px4_msgs__msg__UavcanParameterValue__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__UavcanParameterValue__fini(&array->data[i]);
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

px4_msgs__msg__UavcanParameterValue__Sequence *
px4_msgs__msg__UavcanParameterValue__Sequence__create(size_t size)
{
  px4_msgs__msg__UavcanParameterValue__Sequence * array = (px4_msgs__msg__UavcanParameterValue__Sequence *)malloc(sizeof(px4_msgs__msg__UavcanParameterValue__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__UavcanParameterValue__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__UavcanParameterValue__Sequence__destroy(px4_msgs__msg__UavcanParameterValue__Sequence * array)
{
  if (array) {
    px4_msgs__msg__UavcanParameterValue__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__UavcanParameterValue__Sequence__are_equal(const px4_msgs__msg__UavcanParameterValue__Sequence * lhs, const px4_msgs__msg__UavcanParameterValue__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__UavcanParameterValue__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__UavcanParameterValue__Sequence__copy(
  const px4_msgs__msg__UavcanParameterValue__Sequence * input,
  px4_msgs__msg__UavcanParameterValue__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__UavcanParameterValue);
    px4_msgs__msg__UavcanParameterValue * data =
      (px4_msgs__msg__UavcanParameterValue *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__UavcanParameterValue__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__UavcanParameterValue__fini(&data[i]);
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
    if (!px4_msgs__msg__UavcanParameterValue__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
