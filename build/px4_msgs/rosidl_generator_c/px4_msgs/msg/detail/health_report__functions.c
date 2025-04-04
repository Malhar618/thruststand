// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/HealthReport.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/health_report__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__HealthReport__init(px4_msgs__msg__HealthReport * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // can_arm_mode_flags
  // can_run_mode_flags
  // health_is_present_flags
  // health_warning_flags
  // health_error_flags
  // arming_check_warning_flags
  // arming_check_error_flags
  return true;
}

void
px4_msgs__msg__HealthReport__fini(px4_msgs__msg__HealthReport * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // can_arm_mode_flags
  // can_run_mode_flags
  // health_is_present_flags
  // health_warning_flags
  // health_error_flags
  // arming_check_warning_flags
  // arming_check_error_flags
}

bool
px4_msgs__msg__HealthReport__are_equal(const px4_msgs__msg__HealthReport * lhs, const px4_msgs__msg__HealthReport * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // can_arm_mode_flags
  if (lhs->can_arm_mode_flags != rhs->can_arm_mode_flags) {
    return false;
  }
  // can_run_mode_flags
  if (lhs->can_run_mode_flags != rhs->can_run_mode_flags) {
    return false;
  }
  // health_is_present_flags
  if (lhs->health_is_present_flags != rhs->health_is_present_flags) {
    return false;
  }
  // health_warning_flags
  if (lhs->health_warning_flags != rhs->health_warning_flags) {
    return false;
  }
  // health_error_flags
  if (lhs->health_error_flags != rhs->health_error_flags) {
    return false;
  }
  // arming_check_warning_flags
  if (lhs->arming_check_warning_flags != rhs->arming_check_warning_flags) {
    return false;
  }
  // arming_check_error_flags
  if (lhs->arming_check_error_flags != rhs->arming_check_error_flags) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__HealthReport__copy(
  const px4_msgs__msg__HealthReport * input,
  px4_msgs__msg__HealthReport * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // can_arm_mode_flags
  output->can_arm_mode_flags = input->can_arm_mode_flags;
  // can_run_mode_flags
  output->can_run_mode_flags = input->can_run_mode_flags;
  // health_is_present_flags
  output->health_is_present_flags = input->health_is_present_flags;
  // health_warning_flags
  output->health_warning_flags = input->health_warning_flags;
  // health_error_flags
  output->health_error_flags = input->health_error_flags;
  // arming_check_warning_flags
  output->arming_check_warning_flags = input->arming_check_warning_flags;
  // arming_check_error_flags
  output->arming_check_error_flags = input->arming_check_error_flags;
  return true;
}

px4_msgs__msg__HealthReport *
px4_msgs__msg__HealthReport__create()
{
  px4_msgs__msg__HealthReport * msg = (px4_msgs__msg__HealthReport *)malloc(sizeof(px4_msgs__msg__HealthReport));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__HealthReport));
  bool success = px4_msgs__msg__HealthReport__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__HealthReport__destroy(px4_msgs__msg__HealthReport * msg)
{
  if (msg) {
    px4_msgs__msg__HealthReport__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__HealthReport__Sequence__init(px4_msgs__msg__HealthReport__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__HealthReport * data = NULL;
  if (size) {
    data = (px4_msgs__msg__HealthReport *)calloc(size, sizeof(px4_msgs__msg__HealthReport));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__HealthReport__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__HealthReport__fini(&data[i - 1]);
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
px4_msgs__msg__HealthReport__Sequence__fini(px4_msgs__msg__HealthReport__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__HealthReport__fini(&array->data[i]);
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

px4_msgs__msg__HealthReport__Sequence *
px4_msgs__msg__HealthReport__Sequence__create(size_t size)
{
  px4_msgs__msg__HealthReport__Sequence * array = (px4_msgs__msg__HealthReport__Sequence *)malloc(sizeof(px4_msgs__msg__HealthReport__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__HealthReport__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__HealthReport__Sequence__destroy(px4_msgs__msg__HealthReport__Sequence * array)
{
  if (array) {
    px4_msgs__msg__HealthReport__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__HealthReport__Sequence__are_equal(const px4_msgs__msg__HealthReport__Sequence * lhs, const px4_msgs__msg__HealthReport__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__HealthReport__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__HealthReport__Sequence__copy(
  const px4_msgs__msg__HealthReport__Sequence * input,
  px4_msgs__msg__HealthReport__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__HealthReport);
    px4_msgs__msg__HealthReport * data =
      (px4_msgs__msg__HealthReport *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__HealthReport__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__HealthReport__fini(&data[i]);
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
    if (!px4_msgs__msg__HealthReport__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
