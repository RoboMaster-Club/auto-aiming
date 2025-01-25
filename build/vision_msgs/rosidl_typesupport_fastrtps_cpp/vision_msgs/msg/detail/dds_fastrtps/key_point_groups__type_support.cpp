// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vision_msgs:msg/KeyPointGroups.idl
// generated code does not contain a copyright notice
#include "vision_msgs/msg/detail/key_point_groups__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vision_msgs/msg/detail/key_point_groups__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace vision_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const vision_msgs::msg::KeyPoints &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  vision_msgs::msg::KeyPoints &);
size_t get_serialized_size(
  const vision_msgs::msg::KeyPoints &,
  size_t current_alignment);
size_t
max_serialized_size_KeyPoints(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace vision_msgs


namespace vision_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_msgs
cdr_serialize(
  const vision_msgs::msg::KeyPointGroups & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: groups
  {
    for (size_t i = 0; i < 2; i++) {
      vision_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.groups[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vision_msgs::msg::KeyPointGroups & ros_message)
{
  // Member: groups
  {
    for (size_t i = 0; i < 2; i++) {
      vision_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.groups[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_msgs
get_serialized_size(
  const vision_msgs::msg::KeyPointGroups & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: groups
  {
    size_t array_size = 2;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        vision_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.groups[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_msgs
max_serialized_size_KeyPointGroups(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: groups
  {
    size_t array_size = 2;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        vision_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_KeyPoints(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _KeyPointGroups__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vision_msgs::msg::KeyPointGroups *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _KeyPointGroups__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vision_msgs::msg::KeyPointGroups *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _KeyPointGroups__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vision_msgs::msg::KeyPointGroups *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _KeyPointGroups__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_KeyPointGroups(full_bounded, 0);
}

static message_type_support_callbacks_t _KeyPointGroups__callbacks = {
  "vision_msgs::msg",
  "KeyPointGroups",
  _KeyPointGroups__cdr_serialize,
  _KeyPointGroups__cdr_deserialize,
  _KeyPointGroups__get_serialized_size,
  _KeyPointGroups__max_serialized_size
};

static rosidl_message_type_support_t _KeyPointGroups__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_KeyPointGroups__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vision_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vision_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vision_msgs::msg::KeyPointGroups>()
{
  return &vision_msgs::msg::typesupport_fastrtps_cpp::_KeyPointGroups__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vision_msgs, msg, KeyPointGroups)() {
  return &vision_msgs::msg::typesupport_fastrtps_cpp::_KeyPointGroups__handle;
}

#ifdef __cplusplus
}
#endif
