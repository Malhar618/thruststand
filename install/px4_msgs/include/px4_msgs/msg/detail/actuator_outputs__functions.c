// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/ActuatorOutputs.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/actuator_outputs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__ActuatorOutputs__init(px4_msgs__msg__ActuatorOutputs * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // noutputs
  // output
  return true;
}

void
px4_msgs__msg__ActuatorOutputs__fini(px4_msgs__msg__ActuatorOutputs * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // noutputs
  // output
}

bool
px4_msgs__msg__ActuatorOutputs__are_equal(const px4_msgs__msg__ActuatorOutputs * lhs, const px4_msgs__msg__ActuatorOutputs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // noutputs
  if (lhs->noutputs != rhs->noutputs) {
    return false;
  }
  // output
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->output[i] != rhs->output[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ActuatorOutputs__copy(
  const px4_msgs__msg__ActuatorOutputs * input,
  px4_msgs__msg__ActuatorOutputs * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // noutputs
  output->noutputs = input->noutputs;
  // output
  for (size_t i = 0; i < 16; ++i) {
    output->output[i] = input->output[i];
  }
  return true;
}

px4_msgs__msg__ActuatorOutputs *
px4_msgs__msg__ActuatorOutputs__create()
{
  px4_msgs__msg__ActuatorOutputs * msg = (px4_msgs__msg__ActuatorOutputs *)malloc(sizeof(px4_msgs__msg__ActuatorOutputs));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__ActuatorOutputs));
  bool success = px4_msgs__msg__ActuatorOutputs__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__ActuatorOutputs__destroy(px4_msgs__msg__ActuatorOutputs * msg)
{
  if (msg) {
    px4_msgs__msg__ActuatorOutputs__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__ActuatorOutputs__Sequence__init(px4_msgs__msg__ActuatorOutputs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__ActuatorOutputs * data = NULL;
  if (size) {
    data = (px4_msgs__msg__ActuatorOutputs *)calloc(size, sizeof(px4_msgs__msg__ActuatorOutputs));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__ActuatorOutputs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__ActuatorOutputs__fini(&data[i - 1]);
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
px4_msgs__msg__ActuatorOutputs__Sequence__fini(px4_msgs__msg__ActuatorOutputs__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__ActuatorOutputs__fini(&array->data[i]);
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

px4_msgs__msg__ActuatorOutputs__Sequence *
px4_msgs__msg__ActuatorOutputs__Sequence__create(size_t size)
{
  px4_msgs__msg__ActuatorOutputs__Sequence * array = (px4_msgs__msg__ActuatorOutputs__Sequence *)malloc(sizeof(px4_msgs__msg__ActuatorOutputs__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__ActuatorOutputs__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__ActuatorOutputs__Sequence__destroy(px4_msgs__msg__ActuatorOutputs__Sequence * array)
{
  if (array) {
    px4_msgs__msg__ActuatorOutputs__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__ActuatorOutputs__Sequence__are_equal(const px4_msgs__msg__ActuatorOutputs__Sequence * lhs, const px4_msgs__msg__ActuatorOutputs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__ActuatorOutputs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ActuatorOutputs__Sequence__copy(
  const px4_msgs__msg__ActuatorOutputs__Sequence * input,
  px4_msgs__msg__ActuatorOutputs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__ActuatorOutputs);
    px4_msgs__msg__ActuatorOutputs * data =
      (px4_msgs__msg__ActuatorOutputs *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__ActuatorOutputs__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__ActuatorOutputs__fini(&data[i]);
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
    if (!px4_msgs__msg__ActuatorOutputs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
