// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/GimbalManagerSetAttitude.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/gimbal_manager_set_attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__GimbalManagerSetAttitude__init(px4_msgs__msg__GimbalManagerSetAttitude * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // origin_sysid
  // origin_compid
  // target_system
  // target_component
  // flags
  // gimbal_device_id
  // q
  // angular_velocity_x
  // angular_velocity_y
  // angular_velocity_z
  return true;
}

void
px4_msgs__msg__GimbalManagerSetAttitude__fini(px4_msgs__msg__GimbalManagerSetAttitude * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // origin_sysid
  // origin_compid
  // target_system
  // target_component
  // flags
  // gimbal_device_id
  // q
  // angular_velocity_x
  // angular_velocity_y
  // angular_velocity_z
}

bool
px4_msgs__msg__GimbalManagerSetAttitude__are_equal(const px4_msgs__msg__GimbalManagerSetAttitude * lhs, const px4_msgs__msg__GimbalManagerSetAttitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // origin_sysid
  if (lhs->origin_sysid != rhs->origin_sysid) {
    return false;
  }
  // origin_compid
  if (lhs->origin_compid != rhs->origin_compid) {
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
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  // gimbal_device_id
  if (lhs->gimbal_device_id != rhs->gimbal_device_id) {
    return false;
  }
  // q
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->q[i] != rhs->q[i]) {
      return false;
    }
  }
  // angular_velocity_x
  if (lhs->angular_velocity_x != rhs->angular_velocity_x) {
    return false;
  }
  // angular_velocity_y
  if (lhs->angular_velocity_y != rhs->angular_velocity_y) {
    return false;
  }
  // angular_velocity_z
  if (lhs->angular_velocity_z != rhs->angular_velocity_z) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__GimbalManagerSetAttitude__copy(
  const px4_msgs__msg__GimbalManagerSetAttitude * input,
  px4_msgs__msg__GimbalManagerSetAttitude * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // origin_sysid
  output->origin_sysid = input->origin_sysid;
  // origin_compid
  output->origin_compid = input->origin_compid;
  // target_system
  output->target_system = input->target_system;
  // target_component
  output->target_component = input->target_component;
  // flags
  output->flags = input->flags;
  // gimbal_device_id
  output->gimbal_device_id = input->gimbal_device_id;
  // q
  for (size_t i = 0; i < 4; ++i) {
    output->q[i] = input->q[i];
  }
  // angular_velocity_x
  output->angular_velocity_x = input->angular_velocity_x;
  // angular_velocity_y
  output->angular_velocity_y = input->angular_velocity_y;
  // angular_velocity_z
  output->angular_velocity_z = input->angular_velocity_z;
  return true;
}

px4_msgs__msg__GimbalManagerSetAttitude *
px4_msgs__msg__GimbalManagerSetAttitude__create()
{
  px4_msgs__msg__GimbalManagerSetAttitude * msg = (px4_msgs__msg__GimbalManagerSetAttitude *)malloc(sizeof(px4_msgs__msg__GimbalManagerSetAttitude));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__GimbalManagerSetAttitude));
  bool success = px4_msgs__msg__GimbalManagerSetAttitude__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__GimbalManagerSetAttitude__destroy(px4_msgs__msg__GimbalManagerSetAttitude * msg)
{
  if (msg) {
    px4_msgs__msg__GimbalManagerSetAttitude__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__init(px4_msgs__msg__GimbalManagerSetAttitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__GimbalManagerSetAttitude * data = NULL;
  if (size) {
    data = (px4_msgs__msg__GimbalManagerSetAttitude *)calloc(size, sizeof(px4_msgs__msg__GimbalManagerSetAttitude));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__GimbalManagerSetAttitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__GimbalManagerSetAttitude__fini(&data[i - 1]);
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
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__fini(px4_msgs__msg__GimbalManagerSetAttitude__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__GimbalManagerSetAttitude__fini(&array->data[i]);
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

px4_msgs__msg__GimbalManagerSetAttitude__Sequence *
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__create(size_t size)
{
  px4_msgs__msg__GimbalManagerSetAttitude__Sequence * array = (px4_msgs__msg__GimbalManagerSetAttitude__Sequence *)malloc(sizeof(px4_msgs__msg__GimbalManagerSetAttitude__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__GimbalManagerSetAttitude__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__destroy(px4_msgs__msg__GimbalManagerSetAttitude__Sequence * array)
{
  if (array) {
    px4_msgs__msg__GimbalManagerSetAttitude__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__are_equal(const px4_msgs__msg__GimbalManagerSetAttitude__Sequence * lhs, const px4_msgs__msg__GimbalManagerSetAttitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__GimbalManagerSetAttitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__GimbalManagerSetAttitude__Sequence__copy(
  const px4_msgs__msg__GimbalManagerSetAttitude__Sequence * input,
  px4_msgs__msg__GimbalManagerSetAttitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__GimbalManagerSetAttitude);
    px4_msgs__msg__GimbalManagerSetAttitude * data =
      (px4_msgs__msg__GimbalManagerSetAttitude *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__GimbalManagerSetAttitude__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__GimbalManagerSetAttitude__fini(&data[i]);
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
    if (!px4_msgs__msg__GimbalManagerSetAttitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
