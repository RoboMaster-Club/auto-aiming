// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vision_msgs:msg/YawPitch.idl
// generated code does not contain a copyright notice
#include "vision_msgs/msg/detail/yaw_pitch__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vision_msgs__msg__YawPitch__init(vision_msgs__msg__YawPitch * msg)
{
  if (!msg) {
    return false;
  }
  // pitch
  // yaw
  // dst
  return true;
}

void
vision_msgs__msg__YawPitch__fini(vision_msgs__msg__YawPitch * msg)
{
  if (!msg) {
    return;
  }
  // pitch
  // yaw
  // dst
}

bool
vision_msgs__msg__YawPitch__are_equal(const vision_msgs__msg__YawPitch * lhs, const vision_msgs__msg__YawPitch * rhs)
{
  if (!lhs || !rhs) {
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
  // dst
  if (lhs->dst != rhs->dst) {
    return false;
  }
  return true;
}

bool
vision_msgs__msg__YawPitch__copy(
  const vision_msgs__msg__YawPitch * input,
  vision_msgs__msg__YawPitch * output)
{
  if (!input || !output) {
    return false;
  }
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // dst
  output->dst = input->dst;
  return true;
}

vision_msgs__msg__YawPitch *
vision_msgs__msg__YawPitch__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__YawPitch * msg = (vision_msgs__msg__YawPitch *)allocator.allocate(sizeof(vision_msgs__msg__YawPitch), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vision_msgs__msg__YawPitch));
  bool success = vision_msgs__msg__YawPitch__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vision_msgs__msg__YawPitch__destroy(vision_msgs__msg__YawPitch * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vision_msgs__msg__YawPitch__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vision_msgs__msg__YawPitch__Sequence__init(vision_msgs__msg__YawPitch__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__YawPitch * data = NULL;

  if (size) {
    data = (vision_msgs__msg__YawPitch *)allocator.zero_allocate(size, sizeof(vision_msgs__msg__YawPitch), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vision_msgs__msg__YawPitch__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vision_msgs__msg__YawPitch__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vision_msgs__msg__YawPitch__Sequence__fini(vision_msgs__msg__YawPitch__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vision_msgs__msg__YawPitch__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vision_msgs__msg__YawPitch__Sequence *
vision_msgs__msg__YawPitch__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__YawPitch__Sequence * array = (vision_msgs__msg__YawPitch__Sequence *)allocator.allocate(sizeof(vision_msgs__msg__YawPitch__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vision_msgs__msg__YawPitch__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vision_msgs__msg__YawPitch__Sequence__destroy(vision_msgs__msg__YawPitch__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vision_msgs__msg__YawPitch__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vision_msgs__msg__YawPitch__Sequence__are_equal(const vision_msgs__msg__YawPitch__Sequence * lhs, const vision_msgs__msg__YawPitch__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vision_msgs__msg__YawPitch__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vision_msgs__msg__YawPitch__Sequence__copy(
  const vision_msgs__msg__YawPitch__Sequence * input,
  vision_msgs__msg__YawPitch__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vision_msgs__msg__YawPitch);
    vision_msgs__msg__YawPitch * data =
      (vision_msgs__msg__YawPitch *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vision_msgs__msg__YawPitch__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vision_msgs__msg__YawPitch__fini(&data[i]);
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
    if (!vision_msgs__msg__YawPitch__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
