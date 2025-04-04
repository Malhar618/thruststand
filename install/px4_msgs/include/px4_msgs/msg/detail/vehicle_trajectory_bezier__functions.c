// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleTrajectoryBezier.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_trajectory_bezier__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `control_points`
#include "px4_msgs/msg/detail/trajectory_bezier__functions.h"

bool
px4_msgs__msg__VehicleTrajectoryBezier__init(px4_msgs__msg__VehicleTrajectoryBezier * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // control_points
  for (size_t i = 0; i < 5; ++i) {
    if (!px4_msgs__msg__TrajectoryBezier__init(&msg->control_points[i])) {
      px4_msgs__msg__VehicleTrajectoryBezier__fini(msg);
      return false;
    }
  }
  // bezier_order
  return true;
}

void
px4_msgs__msg__VehicleTrajectoryBezier__fini(px4_msgs__msg__VehicleTrajectoryBezier * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // control_points
  for (size_t i = 0; i < 5; ++i) {
    px4_msgs__msg__TrajectoryBezier__fini(&msg->control_points[i]);
  }
  // bezier_order
}

bool
px4_msgs__msg__VehicleTrajectoryBezier__are_equal(const px4_msgs__msg__VehicleTrajectoryBezier * lhs, const px4_msgs__msg__VehicleTrajectoryBezier * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // control_points
  for (size_t i = 0; i < 5; ++i) {
    if (!px4_msgs__msg__TrajectoryBezier__are_equal(
        &(lhs->control_points[i]), &(rhs->control_points[i])))
    {
      return false;
    }
  }
  // bezier_order
  if (lhs->bezier_order != rhs->bezier_order) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__VehicleTrajectoryBezier__copy(
  const px4_msgs__msg__VehicleTrajectoryBezier * input,
  px4_msgs__msg__VehicleTrajectoryBezier * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // control_points
  for (size_t i = 0; i < 5; ++i) {
    if (!px4_msgs__msg__TrajectoryBezier__copy(
        &(input->control_points[i]), &(output->control_points[i])))
    {
      return false;
    }
  }
  // bezier_order
  output->bezier_order = input->bezier_order;
  return true;
}

px4_msgs__msg__VehicleTrajectoryBezier *
px4_msgs__msg__VehicleTrajectoryBezier__create()
{
  px4_msgs__msg__VehicleTrajectoryBezier * msg = (px4_msgs__msg__VehicleTrajectoryBezier *)malloc(sizeof(px4_msgs__msg__VehicleTrajectoryBezier));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleTrajectoryBezier));
  bool success = px4_msgs__msg__VehicleTrajectoryBezier__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleTrajectoryBezier__destroy(px4_msgs__msg__VehicleTrajectoryBezier * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleTrajectoryBezier__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__init(px4_msgs__msg__VehicleTrajectoryBezier__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleTrajectoryBezier * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleTrajectoryBezier *)calloc(size, sizeof(px4_msgs__msg__VehicleTrajectoryBezier));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleTrajectoryBezier__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleTrajectoryBezier__fini(&data[i - 1]);
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
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__fini(px4_msgs__msg__VehicleTrajectoryBezier__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleTrajectoryBezier__fini(&array->data[i]);
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

px4_msgs__msg__VehicleTrajectoryBezier__Sequence *
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleTrajectoryBezier__Sequence * array = (px4_msgs__msg__VehicleTrajectoryBezier__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleTrajectoryBezier__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleTrajectoryBezier__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__destroy(px4_msgs__msg__VehicleTrajectoryBezier__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleTrajectoryBezier__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__are_equal(const px4_msgs__msg__VehicleTrajectoryBezier__Sequence * lhs, const px4_msgs__msg__VehicleTrajectoryBezier__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__VehicleTrajectoryBezier__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__VehicleTrajectoryBezier__Sequence__copy(
  const px4_msgs__msg__VehicleTrajectoryBezier__Sequence * input,
  px4_msgs__msg__VehicleTrajectoryBezier__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__VehicleTrajectoryBezier);
    px4_msgs__msg__VehicleTrajectoryBezier * data =
      (px4_msgs__msg__VehicleTrajectoryBezier *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__VehicleTrajectoryBezier__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__VehicleTrajectoryBezier__fini(&data[i]);
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
    if (!px4_msgs__msg__VehicleTrajectoryBezier__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
