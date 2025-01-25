// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vision_msgs:msg/KeyPointGroups.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vision_msgs/msg/detail/key_point_groups__rosidl_typesupport_introspection_c.h"
#include "vision_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vision_msgs/msg/detail/key_point_groups__functions.h"
#include "vision_msgs/msg/detail/key_point_groups__struct.h"


// Include directives for member types
// Member `groups`
#include "vision_msgs/msg/key_points.h"
// Member `groups`
#include "vision_msgs/msg/detail/key_points__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vision_msgs__msg__KeyPointGroups__init(message_memory);
}

void KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_fini_function(void * message_memory)
{
  vision_msgs__msg__KeyPointGroups__fini(message_memory);
}

size_t KeyPointGroups__rosidl_typesupport_introspection_c__size_function__KeyPoints__groups(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * KeyPointGroups__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__groups(
  const void * untyped_member, size_t index)
{
  const vision_msgs__msg__KeyPoints ** member =
    (const vision_msgs__msg__KeyPoints **)(untyped_member);
  return &(*member)[index];
}

void * KeyPointGroups__rosidl_typesupport_introspection_c__get_function__KeyPoints__groups(
  void * untyped_member, size_t index)
{
  vision_msgs__msg__KeyPoints ** member =
    (vision_msgs__msg__KeyPoints **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_member_array[1] = {
  {
    "groups",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__KeyPointGroups, groups),  // bytes offset in struct
    NULL,  // default value
    KeyPointGroups__rosidl_typesupport_introspection_c__size_function__KeyPoints__groups,  // size() function pointer
    KeyPointGroups__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__groups,  // get_const(index) function pointer
    KeyPointGroups__rosidl_typesupport_introspection_c__get_function__KeyPoints__groups,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_members = {
  "vision_msgs__msg",  // message namespace
  "KeyPointGroups",  // message name
  1,  // number of fields
  sizeof(vision_msgs__msg__KeyPointGroups),
  KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_member_array,  // message members
  KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_init_function,  // function to initialize message memory (memory has to be allocated)
  KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_type_support_handle = {
  0,
  &KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vision_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, KeyPointGroups)() {
  KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, KeyPoints)();
  if (!KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_type_support_handle.typesupport_identifier) {
    KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &KeyPointGroups__rosidl_typesupport_introspection_c__KeyPointGroups_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
