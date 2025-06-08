// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_msgs:msg/YawPitch.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__YAW_PITCH__TRAITS_HPP_
#define VISION_MSGS__MSG__DETAIL__YAW_PITCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vision_msgs/msg/detail/yaw_pitch__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vision_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const YawPitch & msg,
  std::ostream & out)
{
  out << "{";
  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: dst
  {
    out << "dst: ";
    rosidl_generator_traits::value_to_yaml(msg.dst, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YawPitch & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: dst
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dst: ";
    rosidl_generator_traits::value_to_yaml(msg.dst, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YawPitch & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vision_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vision_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vision_msgs::msg::YawPitch & msg,
  std::ostream & out, size_t indentation = 0)
{
  vision_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vision_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vision_msgs::msg::YawPitch & msg)
{
  return vision_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vision_msgs::msg::YawPitch>()
{
  return "vision_msgs::msg::YawPitch";
}

template<>
inline const char * name<vision_msgs::msg::YawPitch>()
{
  return "vision_msgs/msg/YawPitch";
}

template<>
struct has_fixed_size<vision_msgs::msg::YawPitch>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vision_msgs::msg::YawPitch>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vision_msgs::msg::YawPitch>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISION_MSGS__MSG__DETAIL__YAW_PITCH__TRAITS_HPP_
