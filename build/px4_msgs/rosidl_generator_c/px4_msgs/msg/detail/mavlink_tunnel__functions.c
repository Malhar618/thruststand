// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/MavlinkTunnel.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/mavlink_tunnel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__MavlinkTunnel__init(px4_msgs__msg__MavlinkTunnel * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // payload_type
  // target_system
  // target_component
  // payload_length
  // payload
  return true;
}

void
px4_msgs__msg__MavlinkTunnel__fini(px4_msgs__msg__MavlinkTunnel * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // payload_type
  // target_system
  // target_component
  // payload_length
  // payload
}

bool
px4_msgs__msg__MavlinkTunnel__are_equal(const px4_msgs__msg__MavlinkTunnel * lhs, const px4_msgs__msg__MavlinkTunnel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // payload_type
  if (lhs->payload_type != rhs->payload_type) {
    return false;
  }
  // target_system
  if (lhs->target_system != rhs->target_system) {
    return false;
  }
  // target_component
  if (lhs->target_component != rhs->target_component) {
    return false;
  }
  // payload_length
  if (lhs->payload_length != rhs->payload_length) {
    return false;
  }
  // payload
  for (size_t i = 0; i < 128; ++i) {
    if (lhs->payload[i] != rhs->payload[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__MavlinkTunnel__copy(
  const px4_msgs__msg__MavlinkTunnel * input,
  px4_msgs__msg__MavlinkTunnel * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // payload_type
  output->payload_type = input->payload_type;
  // target_system
  output->target_system = input->target_system;
  // target_component
  output->target_component = input->target_component;
  // payload_length
  output->payload_length = input->payload_length;
  // payload
  for (size_t i = 0; i < 128; ++i) {
    output->payload[i] = input->payload[i];
  }
  return true;
}

px4_msgs__msg__MavlinkTunnel *
px4_msgs__msg__MavlinkTunnel__create()
{
  px4_msgs__msg__MavlinkTunnel * msg = (px4_msgs__msg__MavlinkTunnel *)malloc(sizeof(px4_msgs__msg__MavlinkTunnel));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__MavlinkTunnel));
  bool success = px4_msgs__msg__MavlinkTunnel__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__MavlinkTunnel__destroy(px4_msgs__msg__MavlinkTunnel * msg)
{
  if (msg) {
    px4_msgs__msg__MavlinkTunnel__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__MavlinkTunnel__Sequence__init(px4_msgs__msg__MavlinkTunnel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__MavlinkTunnel * data = NULL;
  if (size) {
    data = (px4_msgs__msg__MavlinkTunnel *)calloc(size, sizeof(px4_msgs__msg__MavlinkTunnel));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__MavlinkTunnel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__MavlinkTunnel__fini(&data[i - 1]);
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
px4_msgs__msg__MavlinkTunnel__Sequence__fini(px4_msgs__msg__MavlinkTunnel__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__MavlinkTunnel__fini(&array->data[i]);
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

px4_msgs__msg__MavlinkTunnel__Sequence *
px4_msgs__msg__MavlinkTunnel__Sequence__create(size_t size)
{
  px4_msgs__msg__MavlinkTunnel__Sequence * array = (px4_msgs__msg__MavlinkTunnel__Sequence *)malloc(sizeof(px4_msgs__msg__MavlinkTunnel__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__MavlinkTunnel__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__MavlinkTunnel__Sequence__destroy(px4_msgs__msg__MavlinkTunnel__Sequence * array)
{
  if (array) {
    px4_msgs__msg__MavlinkTunnel__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__MavlinkTunnel__Sequence__are_equal(const px4_msgs__msg__MavlinkTunnel__Sequence * lhs, const px4_msgs__msg__MavlinkTunnel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__MavlinkTunnel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__MavlinkTunnel__Sequence__copy(
  const px4_msgs__msg__MavlinkTunnel__Sequence * input,
  px4_msgs__msg__MavlinkTunnel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__MavlinkTunnel);
    px4_msgs__msg__MavlinkTunnel * data =
      (px4_msgs__msg__MavlinkTunnel *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__MavlinkTunnel__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__MavlinkTunnel__fini(&data[i]);
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
    if (!px4_msgs__msg__MavlinkTunnel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
