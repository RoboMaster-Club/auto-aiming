// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__TRAITS_HPP_
#define VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vision_msgs/msg/detail/predicted_armor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace vision_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PredictedArmor & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

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

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: x_vel
  {
    out << "x_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_vel, out);
    out << ", ";
  }

  // member: y_vel
  {
    out << "y_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_vel, out);
    out << ", ";
  }

  // member: z_vel
  {
    out << "z_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_vel, out);
    out << ", ";
  }

  // member: fire
  {
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PredictedArmor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

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

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: x_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_vel, out);
    out << "\n";
  }

  // member: y_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_vel, out);
    out << "\n";
  }

  // member: z_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_vel, out);
    out << "\n";
  }

  // member: fire
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PredictedArmor & msg, bool use_flow_style = false)
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
  const vision_msgs::msg::PredictedArmor & msg,
  std::ostream & out, size_t indentation = 0)
{
  vision_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vision_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vision_msgs::msg::PredictedArmor & msg)
{
  return vision_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vision_msgs::msg::PredictedArmor>()
{
  return "vision_msgs::msg::PredictedArmor";
}

template<>
inline const char * name<vision_msgs::msg::PredictedArmor>()
{
  return "vision_msgs/msg/PredictedArmor";
}

template<>
struct has_fixed_size<vision_msgs::msg::PredictedArmor>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<vision_msgs::msg::PredictedArmor>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<vision_msgs::msg::PredictedArmor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__TRAITS_HPP_
