// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vision_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vision_msgs/msg/detail/key_points__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace vision_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void KeyPoints_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vision_msgs::msg::KeyPoints(_init);
}

void KeyPoints_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vision_msgs::msg::KeyPoints *>(message_memory);
  typed_message->~KeyPoints();
}

size_t size_function__KeyPoints__points(const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * get_const_function__KeyPoints__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 8> *>(untyped_member);
  return &member[index];
}

void * get_function__KeyPoints__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 8> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember KeyPoints_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::KeyPoints, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::KeyPoints, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__KeyPoints__points,  // size() function pointer
    get_const_function__KeyPoints__points,  // get_const(index) function pointer
    get_function__KeyPoints__points,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_large_armor",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::KeyPoints, is_large_armor),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers KeyPoints_message_members = {
  "vision_msgs::msg",  // message namespace
  "KeyPoints",  // message name
  3,  // number of fields
  sizeof(vision_msgs::msg::KeyPoints),
  KeyPoints_message_member_array,  // message members
  KeyPoints_init_function,  // function to initialize message memory (memory has to be allocated)
  KeyPoints_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t KeyPoints_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &KeyPoints_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vision_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vision_msgs::msg::KeyPoints>()
{
  return &::vision_msgs::msg::rosidl_typesupport_introspection_cpp::KeyPoints_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vision_msgs, msg, KeyPoints)() {
  return &::vision_msgs::msg::rosidl_typesupport_introspection_cpp::KeyPoints_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
