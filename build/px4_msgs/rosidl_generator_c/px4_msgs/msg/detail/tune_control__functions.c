// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/TuneControl.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/tune_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__TuneControl__init(px4_msgs__msg__TuneControl * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // tune_id
  // tune_override
  // frequency
  // duration
  // silence
  // volume
  return true;
}

void
px4_msgs__msg__TuneControl__fini(px4_msgs__msg__TuneControl * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // tune_id
  // tune_override
  // frequency
  // duration
  // silence
  // volume
}

bool
px4_msgs__msg__TuneControl__are_equal(const px4_msgs__msg__TuneControl * lhs, const px4_msgs__msg__TuneControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // tune_id
  if (lhs->tune_id != rhs->tune_id) {
    return false;
  }
  // tune_override
  if (lhs->tune_override != rhs->tune_override) {
    return false;
  }
  // frequency
  if (lhs->frequency != rhs->frequency) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // silence
  if (lhs->silence != rhs->silence) {
    return false;
  }
  // volume
  if (lhs->volume != rhs->volume) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__TuneControl__copy(
  const px4_msgs__msg__TuneControl * input,
  px4_msgs__msg__TuneControl * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // tune_id
  output->tune_id = input->tune_id;
  // tune_override
  output->tune_override = input->tune_override;
  // frequency
  output->frequency = input->frequency;
  // duration
  output->duration = input->duration;
  // silence
  output->silence = input->silence;
  // volume
  output->volume = input->volume;
  return true;
}

px4_msgs__msg__TuneControl *
px4_msgs__msg__TuneControl__create()
{
  px4_msgs__msg__TuneControl * msg = (px4_msgs__msg__TuneControl *)malloc(sizeof(px4_msgs__msg__TuneControl));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__TuneControl));
  bool success = px4_msgs__msg__TuneControl__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__TuneControl__destroy(px4_msgs__msg__TuneControl * msg)
{
  if (msg) {
    px4_msgs__msg__TuneControl__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__TuneControl__Sequence__init(px4_msgs__msg__TuneControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__TuneControl * data = NULL;
  if (size) {
    data = (px4_msgs__msg__TuneControl *)calloc(size, sizeof(px4_msgs__msg__TuneControl));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__TuneControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__TuneControl__fini(&data[i - 1]);
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
px4_msgs__msg__TuneControl__Sequence__fini(px4_msgs__msg__TuneControl__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__TuneControl__fini(&array->data[i]);
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

px4_msgs__msg__TuneControl__Sequence *
px4_msgs__msg__TuneControl__Sequence__create(size_t size)
{
  px4_msgs__msg__TuneControl__Sequence * array = (px4_msgs__msg__TuneControl__Sequence *)malloc(sizeof(px4_msgs__msg__TuneControl__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__TuneControl__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__TuneControl__Sequence__destroy(px4_msgs__msg__TuneControl__Sequence * array)
{
  if (array) {
    px4_msgs__msg__TuneControl__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__TuneControl__Sequence__are_equal(const px4_msgs__msg__TuneControl__Sequence * lhs, const px4_msgs__msg__TuneControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__TuneControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__TuneControl__Sequence__copy(
  const px4_msgs__msg__TuneControl__Sequence * input,
  px4_msgs__msg__TuneControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__TuneControl);
    px4_msgs__msg__TuneControl * data =
      (px4_msgs__msg__TuneControl *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__TuneControl__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__TuneControl__fini(&data[i]);
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
    if (!px4_msgs__msg__TuneControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
