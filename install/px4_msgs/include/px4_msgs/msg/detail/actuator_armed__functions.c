// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/ActuatorArmed.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/actuator_armed__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__ActuatorArmed__init(px4_msgs__msg__ActuatorArmed * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // armed
  // prearmed
  // ready_to_arm
  // lockdown
  // manual_lockdown
  // force_failsafe
  // in_esc_calibration_mode
  return true;
}

void
px4_msgs__msg__ActuatorArmed__fini(px4_msgs__msg__ActuatorArmed * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // armed
  // prearmed
  // ready_to_arm
  // lockdown
  // manual_lockdown
  // force_failsafe
  // in_esc_calibration_mode
}

bool
px4_msgs__msg__ActuatorArmed__are_equal(const px4_msgs__msg__ActuatorArmed * lhs, const px4_msgs__msg__ActuatorArmed * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // armed
  if (lhs->armed != rhs->armed) {
    return false;
  }
  // prearmed
  if (lhs->prearmed != rhs->prearmed) {
    return false;
  }
  // ready_to_arm
  if (lhs->ready_to_arm != rhs->ready_to_arm) {
    return false;
  }
  // lockdown
  if (lhs->lockdown != rhs->lockdown) {
    return false;
  }
  // manual_lockdown
  if (lhs->manual_lockdown != rhs->manual_lockdown) {
    return false;
  }
  // force_failsafe
  if (lhs->force_failsafe != rhs->force_failsafe) {
    return false;
  }
  // in_esc_calibration_mode
  if (lhs->in_esc_calibration_mode != rhs->in_esc_calibration_mode) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__ActuatorArmed__copy(
  const px4_msgs__msg__ActuatorArmed * input,
  px4_msgs__msg__ActuatorArmed * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // armed
  output->armed = input->armed;
  // prearmed
  output->prearmed = input->prearmed;
  // ready_to_arm
  output->ready_to_arm = input->ready_to_arm;
  // lockdown
  output->lockdown = input->lockdown;
  // manual_lockdown
  output->manual_lockdown = input->manual_lockdown;
  // force_failsafe
  output->force_failsafe = input->force_failsafe;
  // in_esc_calibration_mode
  output->in_esc_calibration_mode = input->in_esc_calibration_mode;
  return true;
}

px4_msgs__msg__ActuatorArmed *
px4_msgs__msg__ActuatorArmed__create()
{
  px4_msgs__msg__ActuatorArmed * msg = (px4_msgs__msg__ActuatorArmed *)malloc(sizeof(px4_msgs__msg__ActuatorArmed));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__ActuatorArmed));
  bool success = px4_msgs__msg__ActuatorArmed__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__ActuatorArmed__destroy(px4_msgs__msg__ActuatorArmed * msg)
{
  if (msg) {
    px4_msgs__msg__ActuatorArmed__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__ActuatorArmed__Sequence__init(px4_msgs__msg__ActuatorArmed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__ActuatorArmed * data = NULL;
  if (size) {
    data = (px4_msgs__msg__ActuatorArmed *)calloc(size, sizeof(px4_msgs__msg__ActuatorArmed));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__ActuatorArmed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__ActuatorArmed__fini(&data[i - 1]);
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
px4_msgs__msg__ActuatorArmed__Sequence__fini(px4_msgs__msg__ActuatorArmed__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__ActuatorArmed__fini(&array->data[i]);
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

px4_msgs__msg__ActuatorArmed__Sequence *
px4_msgs__msg__ActuatorArmed__Sequence__create(size_t size)
{
  px4_msgs__msg__ActuatorArmed__Sequence * array = (px4_msgs__msg__ActuatorArmed__Sequence *)malloc(sizeof(px4_msgs__msg__ActuatorArmed__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__ActuatorArmed__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__ActuatorArmed__Sequence__destroy(px4_msgs__msg__ActuatorArmed__Sequence * array)
{
  if (array) {
    px4_msgs__msg__ActuatorArmed__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__ActuatorArmed__Sequence__are_equal(const px4_msgs__msg__ActuatorArmed__Sequence * lhs, const px4_msgs__msg__ActuatorArmed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__ActuatorArmed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__ActuatorArmed__Sequence__copy(
  const px4_msgs__msg__ActuatorArmed__Sequence * input,
  px4_msgs__msg__ActuatorArmed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__ActuatorArmed);
    px4_msgs__msg__ActuatorArmed * data =
      (px4_msgs__msg__ActuatorArmed *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__ActuatorArmed__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__ActuatorArmed__fini(&data[i]);
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
    if (!px4_msgs__msg__ActuatorArmed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
