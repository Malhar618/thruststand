// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleOpticalFlowVel.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__VehicleOpticalFlowVel__init(px4_msgs__msg__VehicleOpticalFlowVel * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // vel_body
  // vel_ne
  // vel_body_filtered
  // vel_ne_filtered
  // flow_rate_uncompensated
  // flow_rate_compensated
  // gyro_rate
  // gyro_bias
  // ref_gyro
  return true;
}

void
px4_msgs__msg__VehicleOpticalFlowVel__fini(px4_msgs__msg__VehicleOpticalFlowVel * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // vel_body
  // vel_ne
  // vel_body_filtered
  // vel_ne_filtered
  // flow_rate_uncompensated
  // flow_rate_compensated
  // gyro_rate
  // gyro_bias
  // ref_gyro
}

bool
px4_msgs__msg__VehicleOpticalFlowVel__are_equal(const px4_msgs__msg__VehicleOpticalFlowVel * lhs, const px4_msgs__msg__VehicleOpticalFlowVel * rhs)
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
  // vel_body
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->vel_body[i] != rhs->vel_body[i]) {
      return false;
    }
  }
  // vel_ne
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->vel_ne[i] != rhs->vel_ne[i]) {
      return false;
    }
  }
  // vel_body_filtered
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->vel_body_filtered[i] != rhs->vel_body_filtered[i]) {
      return false;
    }
  }
  // vel_ne_filtered
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->vel_ne_filtered[i] != rhs->vel_ne_filtered[i]) {
      return false;
    }
  }
  // flow_rate_uncompensated
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->flow_rate_uncompensated[i] != rhs->flow_rate_uncompensated[i]) {
      return false;
    }
  }
  // flow_rate_compensated
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->flow_rate_compensated[i] != rhs->flow_rate_compensated[i]) {
      return false;
    }
  }
  // gyro_rate
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro_rate[i] != rhs->gyro_rate[i]) {
      return false;
    }
  }
  // gyro_bias
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro_bias[i] != rhs->gyro_bias[i]) {
      return false;
    }
  }
  // ref_gyro
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->ref_gyro[i] != rhs->ref_gyro[i]) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__VehicleOpticalFlowVel__copy(
  const px4_msgs__msg__VehicleOpticalFlowVel * input,
  px4_msgs__msg__VehicleOpticalFlowVel * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // vel_body
  for (size_t i = 0; i < 2; ++i) {
    output->vel_body[i] = input->vel_body[i];
  }
  // vel_ne
  for (size_t i = 0; i < 2; ++i) {
    output->vel_ne[i] = input->vel_ne[i];
  }
  // vel_body_filtered
  for (size_t i = 0; i < 2; ++i) {
    output->vel_body_filtered[i] = input->vel_body_filtered[i];
  }
  // vel_ne_filtered
  for (size_t i = 0; i < 2; ++i) {
    output->vel_ne_filtered[i] = input->vel_ne_filtered[i];
  }
  // flow_rate_uncompensated
  for (size_t i = 0; i < 2; ++i) {
    output->flow_rate_uncompensated[i] = input->flow_rate_uncompensated[i];
  }
  // flow_rate_compensated
  for (size_t i = 0; i < 2; ++i) {
    output->flow_rate_compensated[i] = input->flow_rate_compensated[i];
  }
  // gyro_rate
  for (size_t i = 0; i < 3; ++i) {
    output->gyro_rate[i] = input->gyro_rate[i];
  }
  // gyro_bias
  for (size_t i = 0; i < 3; ++i) {
    output->gyro_bias[i] = input->gyro_bias[i];
  }
  // ref_gyro
  for (size_t i = 0; i < 3; ++i) {
    output->ref_gyro[i] = input->ref_gyro[i];
  }
  return true;
}

px4_msgs__msg__VehicleOpticalFlowVel *
px4_msgs__msg__VehicleOpticalFlowVel__create()
{
  px4_msgs__msg__VehicleOpticalFlowVel * msg = (px4_msgs__msg__VehicleOpticalFlowVel *)malloc(sizeof(px4_msgs__msg__VehicleOpticalFlowVel));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleOpticalFlowVel));
  bool success = px4_msgs__msg__VehicleOpticalFlowVel__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleOpticalFlowVel__destroy(px4_msgs__msg__VehicleOpticalFlowVel * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleOpticalFlowVel__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__init(px4_msgs__msg__VehicleOpticalFlowVel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleOpticalFlowVel * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleOpticalFlowVel *)calloc(size, sizeof(px4_msgs__msg__VehicleOpticalFlowVel));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleOpticalFlowVel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleOpticalFlowVel__fini(&data[i - 1]);
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
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__fini(px4_msgs__msg__VehicleOpticalFlowVel__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleOpticalFlowVel__fini(&array->data[i]);
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

px4_msgs__msg__VehicleOpticalFlowVel__Sequence *
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleOpticalFlowVel__Sequence * array = (px4_msgs__msg__VehicleOpticalFlowVel__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleOpticalFlowVel__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleOpticalFlowVel__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__destroy(px4_msgs__msg__VehicleOpticalFlowVel__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleOpticalFlowVel__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__are_equal(const px4_msgs__msg__VehicleOpticalFlowVel__Sequence * lhs, const px4_msgs__msg__VehicleOpticalFlowVel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__VehicleOpticalFlowVel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__VehicleOpticalFlowVel__Sequence__copy(
  const px4_msgs__msg__VehicleOpticalFlowVel__Sequence * input,
  px4_msgs__msg__VehicleOpticalFlowVel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__VehicleOpticalFlowVel);
    px4_msgs__msg__VehicleOpticalFlowVel * data =
      (px4_msgs__msg__VehicleOpticalFlowVel *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__VehicleOpticalFlowVel__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__VehicleOpticalFlowVel__fini(&data[i]);
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
    if (!px4_msgs__msg__VehicleOpticalFlowVel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
