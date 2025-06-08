// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_H_

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

/// Struct defined in msg/PredictedArmor in the package vision_msgs.
typedef struct vision_msgs__msg__PredictedArmor
{
  std_msgs__msg__Header header;
  /// x position
  double x;
  /// y position
  double y;
  /// z position
  double z;
  /// pitch
  double pitch;
  /// yaw
  double yaw;
  /// roll
  double roll;
  /// x velocity
  double x_vel;
  /// y velocity
  double y_vel;
  /// z velocity
  double z_vel;
  /// true if we have locked on to a target and will fire
  bool fire;
} vision_msgs__msg__PredictedArmor;

// Struct for a sequence of vision_msgs__msg__PredictedArmor.
typedef struct vision_msgs__msg__PredictedArmor__Sequence
{
  vision_msgs__msg__PredictedArmor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__PredictedArmor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_H_
