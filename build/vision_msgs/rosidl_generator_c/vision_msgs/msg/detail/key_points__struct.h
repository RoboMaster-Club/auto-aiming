// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/KeyPoints in the package vision_msgs.
typedef struct vision_msgs__msg__KeyPoints
{
  std_msgs__msg__Header header;
  float points[8];
  bool is_large_armor;
} vision_msgs__msg__KeyPoints;

// Struct for a sequence of vision_msgs__msg__KeyPoints.
typedef struct vision_msgs__msg__KeyPoints__Sequence
{
  vision_msgs__msg__KeyPoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__KeyPoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_
