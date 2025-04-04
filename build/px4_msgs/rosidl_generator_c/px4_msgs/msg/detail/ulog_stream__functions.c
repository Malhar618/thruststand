// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/UlogStream.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/ulog_stream__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__UlogStream__init(px4_msgs__msg__UlogStream * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // length
  // first_message_offset
  // msg_sequence
  // flags
  // data
  return true;
}

void
px4_msgs__msg__UlogStream__fini(px4_msgs__msg__UlogStream * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // length
  // first_message_offset
  // msg_sequence
  // flags
  // data
}

bool
px4_msgs__msg__UlogStream__are_equal(const px4_msgs__msg__UlogStream * lhs, const px4_msgs__msg__UlogStream * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
    return false;
  }
  // first_message_offset
  if (lhs->first_message_offset != rhs->first_message_offset) {
    return false;
  }
  // msg_sequence
  if (lhs->msg_sequence != rhs->msg_sequence) {
    return false;
  }
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  // data
  for (size_t i = 0; i < 249; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__UlogStream__copy(
  const px4_msgs__msg__UlogStream * input,
  px4_msgs__msg__UlogStream * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // length
  output->length = input->length;
  // first_message_offset
  output->first_message_offset = input->first_message_offset;
  // msg_sequence
  output->msg_sequence = input->msg_sequence;
  // flags
  output->flags = input->flags;
  // data
  for (size_t i = 0; i < 249; ++i) {
    output->data[i] = input->data[i];
  }
  return true;
}

px4_msgs__msg__UlogStream *
px4_msgs__msg__UlogStream__create()
{
  px4_msgs__msg__UlogStream * msg = (px4_msgs__msg__UlogStream *)malloc(sizeof(px4_msgs__msg__UlogStream));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__UlogStream));
  bool success = px4_msgs__msg__UlogStream__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__UlogStream__destroy(px4_msgs__msg__UlogStream * msg)
{
  if (msg) {
    px4_msgs__msg__UlogStream__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__UlogStream__Sequence__init(px4_msgs__msg__UlogStream__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__UlogStream * data = NULL;
  if (size) {
    data = (px4_msgs__msg__UlogStream *)calloc(size, sizeof(px4_msgs__msg__UlogStream));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__UlogStream__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__UlogStream__fini(&data[i - 1]);
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
px4_msgs__msg__UlogStream__Sequence__fini(px4_msgs__msg__UlogStream__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__UlogStream__fini(&array->data[i]);
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

px4_msgs__msg__UlogStream__Sequence *
px4_msgs__msg__UlogStream__Sequence__create(size_t size)
{
  px4_msgs__msg__UlogStream__Sequence * array = (px4_msgs__msg__UlogStream__Sequence *)malloc(sizeof(px4_msgs__msg__UlogStream__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__UlogStream__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__UlogStream__Sequence__destroy(px4_msgs__msg__UlogStream__Sequence * array)
{
  if (array) {
    px4_msgs__msg__UlogStream__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__UlogStream__Sequence__are_equal(const px4_msgs__msg__UlogStream__Sequence * lhs, const px4_msgs__msg__UlogStream__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__UlogStream__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__UlogStream__Sequence__copy(
  const px4_msgs__msg__UlogStream__Sequence * input,
  px4_msgs__msg__UlogStream__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__UlogStream);
    px4_msgs__msg__UlogStream * data =
      (px4_msgs__msg__UlogStream *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__UlogStream__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__UlogStream__fini(&data[i]);
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
    if (!px4_msgs__msg__UlogStream__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
