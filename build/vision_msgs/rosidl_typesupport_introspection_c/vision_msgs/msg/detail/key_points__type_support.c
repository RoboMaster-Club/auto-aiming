// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vision_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vision_msgs/msg/detail/key_points__rosidl_typesupport_introspection_c.h"
#include "vision_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vision_msgs/msg/detail/key_points__functions.h"
#include "vision_msgs/msg/detail/key_points__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vision_msgs__msg__KeyPoints__init(message_memory);
}

void KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_fini_function(void * message_memory)
{
  vision_msgs__msg__KeyPoints__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__KeyPoints, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__KeyPoints, points),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_large_armor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__KeyPoints, is_large_armor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_members = {
  "vision_msgs__msg",  // message namespace
  "KeyPoints",  // message name
  3,  // number of fields
  sizeof(vision_msgs__msg__KeyPoints),
  KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_member_array,  // message members
  KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_init_function,  // function to initialize message memory (memory has to be allocated)
  KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle = {
  0,
  &KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vision_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, KeyPoints)() {
  KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle.typesupport_identifier) {
    KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
