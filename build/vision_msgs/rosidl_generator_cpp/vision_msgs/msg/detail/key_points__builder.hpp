// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_
#define VISION_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vision_msgs/msg/detail/key_points__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vision_msgs
{

namespace msg
{

namespace builder
{

class Init_KeyPoints_is_large_armor
{
public:
  explicit Init_KeyPoints_is_large_armor(::vision_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  ::vision_msgs::msg::KeyPoints is_large_armor(::vision_msgs::msg::KeyPoints::_is_large_armor_type arg)
  {
    msg_.is_large_armor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_points
{
public:
  explicit Init_KeyPoints_points(::vision_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  Init_KeyPoints_is_large_armor points(::vision_msgs::msg::KeyPoints::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_KeyPoints_is_large_armor(msg_);
  }

private:
  ::vision_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_header
{
public:
  Init_KeyPoints_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyPoints_points header(::vision_msgs::msg::KeyPoints::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_KeyPoints_points(msg_);
  }

private:
  ::vision_msgs::msg::KeyPoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_msgs::msg::KeyPoints>()
{
  return vision_msgs::msg::builder::Init_KeyPoints_header();
}

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_
