// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice
#include "vision_msgs/msg/detail/predicted_armor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
vision_msgs__msg__PredictedArmor__init(vision_msgs__msg__PredictedArmor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vision_msgs__msg__PredictedArmor__fini(msg);
    return false;
  }
  // x
  // y
  // z
  // pitch
  // yaw
  // roll
  // x_vel
  // y_vel
  // z_vel
  // fire
  return true;
}

void
vision_msgs__msg__PredictedArmor__fini(vision_msgs__msg__PredictedArmor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x
  // y
  // z
  // pitch
  // yaw
  // roll
  // x_vel
  // y_vel
  // z_vel
  // fire
}

bool
vision_msgs__msg__PredictedArmor__are_equal(const vision_msgs__msg__PredictedArmor * lhs, const vision_msgs__msg__PredictedArmor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
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
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // x_vel
  if (lhs->x_vel != rhs->x_vel) {
    return false;
  }
  // y_vel
  if (lhs->y_vel != rhs->y_vel) {
    return false;
  }
  // z_vel
  if (lhs->z_vel != rhs->z_vel) {
    return false;
  }
  // fire
  if (lhs->fire != rhs->fire) {
    return false;
  }
  return true;
}

bool
vision_msgs__msg__PredictedArmor__copy(
  const vision_msgs__msg__PredictedArmor * input,
  vision_msgs__msg__PredictedArmor * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // roll
  output->roll = input->roll;
  // x_vel
  output->x_vel = input->x_vel;
  // y_vel
  output->y_vel = input->y_vel;
  // z_vel
  output->z_vel = input->z_vel;
  // fire
  output->fire = input->fire;
  return true;
}

vision_msgs__msg__PredictedArmor *
vision_msgs__msg__PredictedArmor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__PredictedArmor * msg = (vision_msgs__msg__PredictedArmor *)allocator.allocate(sizeof(vision_msgs__msg__PredictedArmor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vision_msgs__msg__PredictedArmor));
  bool success = vision_msgs__msg__PredictedArmor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vision_msgs__msg__PredictedArmor__destroy(vision_msgs__msg__PredictedArmor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vision_msgs__msg__PredictedArmor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vision_msgs__msg__PredictedArmor__Sequence__init(vision_msgs__msg__PredictedArmor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__PredictedArmor * data = NULL;

  if (size) {
    data = (vision_msgs__msg__PredictedArmor *)allocator.zero_allocate(size, sizeof(vision_msgs__msg__PredictedArmor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vision_msgs__msg__PredictedArmor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vision_msgs__msg__PredictedArmor__fini(&data[i - 1]);
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
vision_msgs__msg__PredictedArmor__Sequence__fini(vision_msgs__msg__PredictedArmor__Sequence * array)
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
      vision_msgs__msg__PredictedArmor__fini(&array->data[i]);
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

vision_msgs__msg__PredictedArmor__Sequence *
vision_msgs__msg__PredictedArmor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__PredictedArmor__Sequence * array = (vision_msgs__msg__PredictedArmor__Sequence *)allocator.allocate(sizeof(vision_msgs__msg__PredictedArmor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vision_msgs__msg__PredictedArmor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vision_msgs__msg__PredictedArmor__Sequence__destroy(vision_msgs__msg__PredictedArmor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vision_msgs__msg__PredictedArmor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vision_msgs__msg__PredictedArmor__Sequence__are_equal(const vision_msgs__msg__PredictedArmor__Sequence * lhs, const vision_msgs__msg__PredictedArmor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vision_msgs__msg__PredictedArmor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vision_msgs__msg__PredictedArmor__Sequence__copy(
  const vision_msgs__msg__PredictedArmor__Sequence * input,
  vision_msgs__msg__PredictedArmor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vision_msgs__msg__PredictedArmor);
    vision_msgs__msg__PredictedArmor * data =
      (vision_msgs__msg__PredictedArmor *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vision_msgs__msg__PredictedArmor__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vision_msgs__msg__PredictedArmor__fini(&data[i]);
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
    if (!vision_msgs__msg__PredictedArmor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
