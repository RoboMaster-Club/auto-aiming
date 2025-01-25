// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__TRAITS_HPP_
#define VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__TRAITS_HPP_

#include "vision_msgs/msg/detail/predicted_armor__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

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
