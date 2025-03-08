// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_msgs:msg/YawPitch.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__YAW_PITCH__BUILDER_HPP_
#define VISION_MSGS__MSG__DETAIL__YAW_PITCH__BUILDER_HPP_

#include "vision_msgs/msg/detail/yaw_pitch__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vision_msgs
{

namespace msg
{

namespace builder
{

class Init_YawPitch_dst
{
public:
  explicit Init_YawPitch_dst(::vision_msgs::msg::YawPitch & msg)
  : msg_(msg)
  {}
  ::vision_msgs::msg::YawPitch dst(::vision_msgs::msg::YawPitch::_dst_type arg)
  {
    msg_.dst = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_msgs::msg::YawPitch msg_;
};

class Init_YawPitch_yaw
{
public:
  explicit Init_YawPitch_yaw(::vision_msgs::msg::YawPitch & msg)
  : msg_(msg)
  {}
  Init_YawPitch_dst yaw(::vision_msgs::msg::YawPitch::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_YawPitch_dst(msg_);
  }

private:
  ::vision_msgs::msg::YawPitch msg_;
};

class Init_YawPitch_pitch
{
public:
  Init_YawPitch_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YawPitch_yaw pitch(::vision_msgs::msg::YawPitch::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_YawPitch_yaw(msg_);
  }

private:
  ::vision_msgs::msg::YawPitch msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_msgs::msg::YawPitch>()
{
  return vision_msgs::msg::builder::Init_YawPitch_pitch();
}

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__YAW_PITCH__BUILDER_HPP_
