// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/NpfgStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/npfg_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__NpfgStatus__init(px4_msgs__msg__NpfgStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // wind_est_valid
  // lat_accel
  // lat_accel_ff
  // bearing_feas
  // bearing_feas_on_track
  // signed_track_error
  // track_error_bound
  // airspeed_ref
  // bearing
  // heading_ref
  // min_ground_speed_ref
  // adapted_period
  // p_gain
  // time_const
  // can_run_factor
  return true;
}

void
px4_msgs__msg__NpfgStatus__fini(px4_msgs__msg__NpfgStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // wind_est_valid
  // lat_accel
  // lat_accel_ff
  // bearing_feas
  // bearing_feas_on_track
  // signed_track_error
  // track_error_bound
  // airspeed_ref
  // bearing
  // heading_ref
  // min_ground_speed_ref
  // adapted_period
  // p_gain
  // time_const
  // can_run_factor
}

bool
px4_msgs__msg__NpfgStatus__are_equal(const px4_msgs__msg__NpfgStatus * lhs, const px4_msgs__msg__NpfgStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // wind_est_valid
  if (lhs->wind_est_valid != rhs->wind_est_valid) {
    return false;
  }
  // lat_accel
  if (lhs->lat_accel != rhs->lat_accel) {
    return false;
  }
  // lat_accel_ff
  if (lhs->lat_accel_ff != rhs->lat_accel_ff) {
    return false;
  }
  // bearing_feas
  if (lhs->bearing_feas != rhs->bearing_feas) {
    return false;
  }
  // bearing_feas_on_track
  if (lhs->bearing_feas_on_track != rhs->bearing_feas_on_track) {
    return false;
  }
  // signed_track_error
  if (lhs->signed_track_error != rhs->signed_track_error) {
    return false;
  }
  // track_error_bound
  if (lhs->track_error_bound != rhs->track_error_bound) {
    return false;
  }
  // airspeed_ref
  if (lhs->airspeed_ref != rhs->airspeed_ref) {
    return false;
  }
  // bearing
  if (lhs->bearing != rhs->bearing) {
    return false;
  }
  // heading_ref
  if (lhs->heading_ref != rhs->heading_ref) {
    return false;
  }
  // min_ground_speed_ref
  if (lhs->min_ground_speed_ref != rhs->min_ground_speed_ref) {
    return false;
  }
  // adapted_period
  if (lhs->adapted_period != rhs->adapted_period) {
    return false;
  }
  // p_gain
  if (lhs->p_gain != rhs->p_gain) {
    return false;
  }
  // time_const
  if (lhs->time_const != rhs->time_const) {
    return false;
  }
  // can_run_factor
  if (lhs->can_run_factor != rhs->can_run_factor) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__NpfgStatus__copy(
  const px4_msgs__msg__NpfgStatus * input,
  px4_msgs__msg__NpfgStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // wind_est_valid
  output->wind_est_valid = input->wind_est_valid;
  // lat_accel
  output->lat_accel = input->lat_accel;
  // lat_accel_ff
  output->lat_accel_ff = input->lat_accel_ff;
  // bearing_feas
  output->bearing_feas = input->bearing_feas;
  // bearing_feas_on_track
  output->bearing_feas_on_track = input->bearing_feas_on_track;
  // signed_track_error
  output->signed_track_error = input->signed_track_error;
  // track_error_bound
  output->track_error_bound = input->track_error_bound;
  // airspeed_ref
  output->airspeed_ref = input->airspeed_ref;
  // bearing
  output->bearing = input->bearing;
  // heading_ref
  output->heading_ref = input->heading_ref;
  // min_ground_speed_ref
  output->min_ground_speed_ref = input->min_ground_speed_ref;
  // adapted_period
  output->adapted_period = input->adapted_period;
  // p_gain
  output->p_gain = input->p_gain;
  // time_const
  output->time_const = input->time_const;
  // can_run_factor
  output->can_run_factor = input->can_run_factor;
  return true;
}

px4_msgs__msg__NpfgStatus *
px4_msgs__msg__NpfgStatus__create()
{
  px4_msgs__msg__NpfgStatus * msg = (px4_msgs__msg__NpfgStatus *)malloc(sizeof(px4_msgs__msg__NpfgStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__NpfgStatus));
  bool success = px4_msgs__msg__NpfgStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__NpfgStatus__destroy(px4_msgs__msg__NpfgStatus * msg)
{
  if (msg) {
    px4_msgs__msg__NpfgStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__NpfgStatus__Sequence__init(px4_msgs__msg__NpfgStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__NpfgStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__NpfgStatus *)calloc(size, sizeof(px4_msgs__msg__NpfgStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__NpfgStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__NpfgStatus__fini(&data[i - 1]);
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
px4_msgs__msg__NpfgStatus__Sequence__fini(px4_msgs__msg__NpfgStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__NpfgStatus__fini(&array->data[i]);
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

px4_msgs__msg__NpfgStatus__Sequence *
px4_msgs__msg__NpfgStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__NpfgStatus__Sequence * array = (px4_msgs__msg__NpfgStatus__Sequence *)malloc(sizeof(px4_msgs__msg__NpfgStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__NpfgStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__NpfgStatus__Sequence__destroy(px4_msgs__msg__NpfgStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__NpfgStatus__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__NpfgStatus__Sequence__are_equal(const px4_msgs__msg__NpfgStatus__Sequence * lhs, const px4_msgs__msg__NpfgStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__NpfgStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__NpfgStatus__Sequence__copy(
  const px4_msgs__msg__NpfgStatus__Sequence * input,
  px4_msgs__msg__NpfgStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__NpfgStatus);
    px4_msgs__msg__NpfgStatus * data =
      (px4_msgs__msg__NpfgStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__NpfgStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__NpfgStatus__fini(&data[i]);
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
    if (!px4_msgs__msg__NpfgStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
