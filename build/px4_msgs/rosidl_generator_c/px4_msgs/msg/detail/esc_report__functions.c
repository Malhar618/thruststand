// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/EscReport.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/esc_report__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__EscReport__init(px4_msgs__msg__EscReport * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // esc_errorcount
  // esc_rpm
  // esc_voltage
  // esc_current
  // esc_temperature
  // esc_address
  // esc_cmdcount
  // esc_state
  // actuator_function
  // failures
  // esc_power
  return true;
}

void
px4_msgs__msg__EscReport__fini(px4_msgs__msg__EscReport * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // esc_errorcount
  // esc_rpm
  // esc_voltage
  // esc_current
  // esc_temperature
  // esc_address
  // esc_cmdcount
  // esc_state
  // actuator_function
  // failures
  // esc_power
}

bool
px4_msgs__msg__EscReport__are_equal(const px4_msgs__msg__EscReport * lhs, const px4_msgs__msg__EscReport * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // esc_errorcount
  if (lhs->esc_errorcount != rhs->esc_errorcount) {
    return false;
  }
  // esc_rpm
  if (lhs->esc_rpm != rhs->esc_rpm) {
    return false;
  }
  // esc_voltage
  if (lhs->esc_voltage != rhs->esc_voltage) {
    return false;
  }
  // esc_current
  if (lhs->esc_current != rhs->esc_current) {
    return false;
  }
  // esc_temperature
  if (lhs->esc_temperature != rhs->esc_temperature) {
    return false;
  }
  // esc_address
  if (lhs->esc_address != rhs->esc_address) {
    return false;
  }
  // esc_cmdcount
  if (lhs->esc_cmdcount != rhs->esc_cmdcount) {
    return false;
  }
  // esc_state
  if (lhs->esc_state != rhs->esc_state) {
    return false;
  }
  // actuator_function
  if (lhs->actuator_function != rhs->actuator_function) {
    return false;
  }
  // failures
  if (lhs->failures != rhs->failures) {
    return false;
  }
  // esc_power
  if (lhs->esc_power != rhs->esc_power) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__EscReport__copy(
  const px4_msgs__msg__EscReport * input,
  px4_msgs__msg__EscReport * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // esc_errorcount
  output->esc_errorcount = input->esc_errorcount;
  // esc_rpm
  output->esc_rpm = input->esc_rpm;
  // esc_voltage
  output->esc_voltage = input->esc_voltage;
  // esc_current
  output->esc_current = input->esc_current;
  // esc_temperature
  output->esc_temperature = input->esc_temperature;
  // esc_address
  output->esc_address = input->esc_address;
  // esc_cmdcount
  output->esc_cmdcount = input->esc_cmdcount;
  // esc_state
  output->esc_state = input->esc_state;
  // actuator_function
  output->actuator_function = input->actuator_function;
  // failures
  output->failures = input->failures;
  // esc_power
  output->esc_power = input->esc_power;
  return true;
}

px4_msgs__msg__EscReport *
px4_msgs__msg__EscReport__create()
{
  px4_msgs__msg__EscReport * msg = (px4_msgs__msg__EscReport *)malloc(sizeof(px4_msgs__msg__EscReport));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__EscReport));
  bool success = px4_msgs__msg__EscReport__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__EscReport__destroy(px4_msgs__msg__EscReport * msg)
{
  if (msg) {
    px4_msgs__msg__EscReport__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__EscReport__Sequence__init(px4_msgs__msg__EscReport__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__EscReport * data = NULL;
  if (size) {
    data = (px4_msgs__msg__EscReport *)calloc(size, sizeof(px4_msgs__msg__EscReport));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__EscReport__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__EscReport__fini(&data[i - 1]);
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
px4_msgs__msg__EscReport__Sequence__fini(px4_msgs__msg__EscReport__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__EscReport__fini(&array->data[i]);
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

px4_msgs__msg__EscReport__Sequence *
px4_msgs__msg__EscReport__Sequence__create(size_t size)
{
  px4_msgs__msg__EscReport__Sequence * array = (px4_msgs__msg__EscReport__Sequence *)malloc(sizeof(px4_msgs__msg__EscReport__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__EscReport__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__EscReport__Sequence__destroy(px4_msgs__msg__EscReport__Sequence * array)
{
  if (array) {
    px4_msgs__msg__EscReport__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__EscReport__Sequence__are_equal(const px4_msgs__msg__EscReport__Sequence * lhs, const px4_msgs__msg__EscReport__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__EscReport__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__EscReport__Sequence__copy(
  const px4_msgs__msg__EscReport__Sequence * input,
  px4_msgs__msg__EscReport__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__EscReport);
    px4_msgs__msg__EscReport * data =
      (px4_msgs__msg__EscReport *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__EscReport__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__EscReport__fini(&data[i]);
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
    if (!px4_msgs__msg__EscReport__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
