// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/ManualControlSetpoint.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/manual_control_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__ManualControlSetpoint__init(px4_msgs__msg__ManualControlSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // valid
  // data_source
  // roll
  // pitch
  // yaw
  // throttle
  // flaps
  // aux1
  // aux2
  // aux3
  // aux4
  // aux5
  // aux6
  // sticks_moving
  // buttons
  return true;
}

void
px4_msgs__msg__ManualControlSetpoint__fini(px4_msgs__msg__ManualControlSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // valid
  // data_source
  // roll
  // pitch
  // yaw
  // throttle
  // flaps
  // aux1
  // aux2
  // aux3
  // aux4
  // aux5
  // aux6
  // sticks_moving
  // buttons
}

bool
px4_msgs__msg__ManualControlSetpoint__are_equal(const px4_msgs__msg__ManualControlSetpoint * lhs, const px4_msgs__msg__ManualControlSetpoint * rhs)
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
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // data_source
  if (lhs->data_source != rhs->data_source) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // flaps
  if (lhs->flaps != rhs->flaps) {
    return false;
  }
  // aux1
  if (lhs->aux1 != rhs->aux1) {
    return false;
  }
  // aux2
  if (lhs->aux2 != rhs->aux2) {
    return false;
  }
  // aux3
  if (lhs->aux3 != rhs->aux3) {
    return false;
  }
  // aux4
  if (lhs->aux4 != rhs->aux4) {
    return false;
  }
  // aux5
  if (lhs->aux5 != rhs->aux5) {
    return false;
  }
  // aux6
  if (lhs->aux6 != rhs->aux6) {
    return false;
  }
  // sticks_moving
  if (lhs->sticks_moving != rhs->sticks_moving) {
    return false;
  }
  // buttons
  if (lhs->buttons != rhs->buttons) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__ManualControlSetpoint__copy(
  const px4_msgs__msg__ManualControlSetpoint * input,
  px4_msgs__msg__ManualControlSetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // valid
  output->valid = input->valid;
  // data_source
  output->data_source = input->data_source;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // throttle
  output->throttle = input->throttle;
  // flaps
  output->flaps = input->flaps;
  // aux1
  output->aux1 = input->aux1;
  // aux2
  output->aux2 = input->aux2;
  // aux3
  output->aux3 = input->aux3;
  // aux4
  output->aux4 = input->aux4;
  // aux5
  output->aux5 = input->aux5;
  // aux6
  output->aux6 = input->aux6;
  // sticks_moving
  output->sticks_moving = input->sticks_moving;
  // buttons
  output->buttons = input->buttons;
  return true;
}

px4_msgs__msg__ManualControlSetpoint *
px4_msgs__msg__ManualControlSetpoint__create()
{
  px4_msgs__msg__ManualControlSetpoint * msg = (px4_msgs__msg__ManualControlSetpoint *)malloc(sizeof(px4_msgs__msg__ManualControlSetpoint));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__ManualControlSetpoint));
  bool success = px4_msgs__msg__ManualControlSetpoint__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__ManualControlSetpoint__destroy(px4_msgs__msg__ManualControlSetpoint * msg)
{
  if (msg) {
    px4_msgs__msg__ManualControlSetpoint__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__ManualControlSetpoint__Sequence__init(px4_msgs__msg__ManualControlSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__ManualControlSetpoint * data = NULL;
  if (size) {
    data = (px4_msgs__msg__ManualControlSetpoint *)calloc(size, sizeof(px4_msgs__msg__ManualControlSetpoint));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__ManualControlSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__ManualControlSetpoint__fini(&data[i - 1]);
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
px4_msgs__msg__ManualControlSetpoint__Sequence__fini(px4_msgs__msg__ManualControlSetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__ManualControlSetpoint__fini(&array->data[i]);
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

px4_msgs__msg__ManualControlSetpoint__Sequence *
px4_msgs__msg__ManualControlSetpoint__Sequence__create(size_t size)
{
  px4_msgs__msg__ManualControlSetpoint__Sequence * array = (px4_msgs__msg__ManualControlSetpoint__Sequence *)malloc(sizeof(px4_msgs__msg__ManualControlSetpoint__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__ManualControlSetpoint__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__ManualControlSetpoint__Sequence__destroy(px4_msgs__msg__ManualControlSetpoint__Sequence * array)
{
  if (array) {
    px4_msgs__msg__ManualControlSetpoint__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__ManualControlSetpoint__Sequence__are_equal(const px4_msgs__msg__ManualControlSetpoint__Sequence * lhs, const px4_msgs__msg__ManualControlSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__ManualControlSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ManualControlSetpoint__Sequence__copy(
  const px4_msgs__msg__ManualControlSetpoint__Sequence * input,
  px4_msgs__msg__ManualControlSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__ManualControlSetpoint);
    px4_msgs__msg__ManualControlSetpoint * data =
      (px4_msgs__msg__ManualControlSetpoint *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__ManualControlSetpoint__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__ManualControlSetpoint__fini(&data[i]);
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
    if (!px4_msgs__msg__ManualControlSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
