// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/KeyPointGroups.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'groups'
#include "vision_msgs/msg/detail/key_points__struct.h"

// Struct defined in msg/KeyPointGroups in the package vision_msgs.
typedef struct vision_msgs__msg__KeyPointGroups
{
  vision_msgs__msg__KeyPoints groups[2];
} vision_msgs__msg__KeyPointGroups;

// Struct for a sequence of vision_msgs__msg__KeyPointGroups.
typedef struct vision_msgs__msg__KeyPointGroups__Sequence
{
  vision_msgs__msg__KeyPointGroups * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__KeyPointGroups__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_H_
