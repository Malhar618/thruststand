// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/EstimatorGpsStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_gps_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__EstimatorGpsStatus__init(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // checks_passed
  // check_fail_gps_fix
  // check_fail_min_sat_count
  // check_fail_max_pdop
  // check_fail_max_horz_err
  // check_fail_max_vert_err
  // check_fail_max_spd_err
  // check_fail_max_horz_drift
  // check_fail_max_vert_drift
  // check_fail_max_horz_spd_err
  // check_fail_max_vert_spd_err
  // check_fail_spoofed_gps
  // position_drift_rate_horizontal_m_s
  // position_drift_rate_vertical_m_s
  // filtered_horizontal_speed_m_s
  return true;
}

void
px4_msgs__msg__EstimatorGpsStatus__fini(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // checks_passed
  // check_fail_gps_fix
  // check_fail_min_sat_count
  // check_fail_max_pdop
  // check_fail_max_horz_err
  // check_fail_max_vert_err
  // check_fail_max_spd_err
  // check_fail_max_horz_drift
  // check_fail_max_vert_drift
  // check_fail_max_horz_spd_err
  // check_fail_max_vert_spd_err
  // check_fail_spoofed_gps
  // position_drift_rate_horizontal_m_s
  // position_drift_rate_vertical_m_s
  // filtered_horizontal_speed_m_s
}

bool
px4_msgs__msg__EstimatorGpsStatus__are_equal(const px4_msgs__msg__EstimatorGpsStatus * lhs, const px4_msgs__msg__EstimatorGpsStatus * rhs)
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
  // checks_passed
  if (lhs->checks_passed != rhs->checks_passed) {
    return false;
  }
  // check_fail_gps_fix
  if (lhs->check_fail_gps_fix != rhs->check_fail_gps_fix) {
    return false;
  }
  // check_fail_min_sat_count
  if (lhs->check_fail_min_sat_count != rhs->check_fail_min_sat_count) {
    return false;
  }
  // check_fail_max_pdop
  if (lhs->check_fail_max_pdop != rhs->check_fail_max_pdop) {
    return false;
  }
  // check_fail_max_horz_err
  if (lhs->check_fail_max_horz_err != rhs->check_fail_max_horz_err) {
    return false;
  }
  // check_fail_max_vert_err
  if (lhs->check_fail_max_vert_err != rhs->check_fail_max_vert_err) {
    return false;
  }
  // check_fail_max_spd_err
  if (lhs->check_fail_max_spd_err != rhs->check_fail_max_spd_err) {
    return false;
  }
  // check_fail_max_horz_drift
  if (lhs->check_fail_max_horz_drift != rhs->check_fail_max_horz_drift) {
    return false;
  }
  // check_fail_max_vert_drift
  if (lhs->check_fail_max_vert_drift != rhs->check_fail_max_vert_drift) {
    return false;
  }
  // check_fail_max_horz_spd_err
  if (lhs->check_fail_max_horz_spd_err != rhs->check_fail_max_horz_spd_err) {
    return false;
  }
  // check_fail_max_vert_spd_err
  if (lhs->check_fail_max_vert_spd_err != rhs->check_fail_max_vert_spd_err) {
    return false;
  }
  // check_fail_spoofed_gps
  if (lhs->check_fail_spoofed_gps != rhs->check_fail_spoofed_gps) {
    return false;
  }
  // position_drift_rate_horizontal_m_s
  if (lhs->position_drift_rate_horizontal_m_s != rhs->position_drift_rate_horizontal_m_s) {
    return false;
  }
  // position_drift_rate_vertical_m_s
  if (lhs->position_drift_rate_vertical_m_s != rhs->position_drift_rate_vertical_m_s) {
    return false;
  }
  // filtered_horizontal_speed_m_s
  if (lhs->filtered_horizontal_speed_m_s != rhs->filtered_horizontal_speed_m_s) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__EstimatorGpsStatus__copy(
  const px4_msgs__msg__EstimatorGpsStatus * input,
  px4_msgs__msg__EstimatorGpsStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // checks_passed
  output->checks_passed = input->checks_passed;
  // check_fail_gps_fix
  output->check_fail_gps_fix = input->check_fail_gps_fix;
  // check_fail_min_sat_count
  output->check_fail_min_sat_count = input->check_fail_min_sat_count;
  // check_fail_max_pdop
  output->check_fail_max_pdop = input->check_fail_max_pdop;
  // check_fail_max_horz_err
  output->check_fail_max_horz_err = input->check_fail_max_horz_err;
  // check_fail_max_vert_err
  output->check_fail_max_vert_err = input->check_fail_max_vert_err;
  // check_fail_max_spd_err
  output->check_fail_max_spd_err = input->check_fail_max_spd_err;
  // check_fail_max_horz_drift
  output->check_fail_max_horz_drift = input->check_fail_max_horz_drift;
  // check_fail_max_vert_drift
  output->check_fail_max_vert_drift = input->check_fail_max_vert_drift;
  // check_fail_max_horz_spd_err
  output->check_fail_max_horz_spd_err = input->check_fail_max_horz_spd_err;
  // check_fail_max_vert_spd_err
  output->check_fail_max_vert_spd_err = input->check_fail_max_vert_spd_err;
  // check_fail_spoofed_gps
  output->check_fail_spoofed_gps = input->check_fail_spoofed_gps;
  // position_drift_rate_horizontal_m_s
  output->position_drift_rate_horizontal_m_s = input->position_drift_rate_horizontal_m_s;
  // position_drift_rate_vertical_m_s
  output->position_drift_rate_vertical_m_s = input->position_drift_rate_vertical_m_s;
  // filtered_horizontal_speed_m_s
  output->filtered_horizontal_speed_m_s = input->filtered_horizontal_speed_m_s;
  return true;
}

px4_msgs__msg__EstimatorGpsStatus *
px4_msgs__msg__EstimatorGpsStatus__create()
{
  px4_msgs__msg__EstimatorGpsStatus * msg = (px4_msgs__msg__EstimatorGpsStatus *)malloc(sizeof(px4_msgs__msg__EstimatorGpsStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__EstimatorGpsStatus));
  bool success = px4_msgs__msg__EstimatorGpsStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__EstimatorGpsStatus__destroy(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (msg) {
    px4_msgs__msg__EstimatorGpsStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__EstimatorGpsStatus__Sequence__init(px4_msgs__msg__EstimatorGpsStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__EstimatorGpsStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__EstimatorGpsStatus *)calloc(size, sizeof(px4_msgs__msg__EstimatorGpsStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__EstimatorGpsStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__EstimatorGpsStatus__fini(&data[i - 1]);
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
px4_msgs__msg__EstimatorGpsStatus__Sequence__fini(px4_msgs__msg__EstimatorGpsStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__EstimatorGpsStatus__fini(&array->data[i]);
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

px4_msgs__msg__EstimatorGpsStatus__Sequence *
px4_msgs__msg__EstimatorGpsStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__EstimatorGpsStatus__Sequence * array = (px4_msgs__msg__EstimatorGpsStatus__Sequence *)malloc(sizeof(px4_msgs__msg__EstimatorGpsStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__EstimatorGpsStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__EstimatorGpsStatus__Sequence__destroy(px4_msgs__msg__EstimatorGpsStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__EstimatorGpsStatus__Sequence__fini(array);
  }
  free(array);
}

bool
px4_msgs__msg__EstimatorGpsStatus__Sequence__are_equal(const px4_msgs__msg__EstimatorGpsStatus__Sequence * lhs, const px4_msgs__msg__EstimatorGpsStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__EstimatorGpsStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__EstimatorGpsStatus__Sequence__copy(
  const px4_msgs__msg__EstimatorGpsStatus__Sequence * input,
  px4_msgs__msg__EstimatorGpsStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__EstimatorGpsStatus);
    px4_msgs__msg__EstimatorGpsStatus * data =
      (px4_msgs__msg__EstimatorGpsStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__EstimatorGpsStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__EstimatorGpsStatus__fini(&data[i]);
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
    if (!px4_msgs__msg__EstimatorGpsStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
