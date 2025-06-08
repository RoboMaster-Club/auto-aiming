// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__BUILDER_HPP_
#define VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vision_msgs/msg/detail/predicted_armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vision_msgs
{

namespace msg
{

namespace builder
{

class Init_PredictedArmor_fire
{
public:
  explicit Init_PredictedArmor_fire(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  ::vision_msgs::msg::PredictedArmor fire(::vision_msgs::msg::PredictedArmor::_fire_type arg)
  {
    msg_.fire = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_z_vel
{
public:
  explicit Init_PredictedArmor_z_vel(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_fire z_vel(::vision_msgs::msg::PredictedArmor::_z_vel_type arg)
  {
    msg_.z_vel = std::move(arg);
    return Init_PredictedArmor_fire(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_y_vel
{
public:
  explicit Init_PredictedArmor_y_vel(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_z_vel y_vel(::vision_msgs::msg::PredictedArmor::_y_vel_type arg)
  {
    msg_.y_vel = std::move(arg);
    return Init_PredictedArmor_z_vel(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_x_vel
{
public:
  explicit Init_PredictedArmor_x_vel(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_y_vel x_vel(::vision_msgs::msg::PredictedArmor::_x_vel_type arg)
  {
    msg_.x_vel = std::move(arg);
    return Init_PredictedArmor_y_vel(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_roll
{
public:
  explicit Init_PredictedArmor_roll(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_x_vel roll(::vision_msgs::msg::PredictedArmor::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_PredictedArmor_x_vel(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_yaw
{
public:
  explicit Init_PredictedArmor_yaw(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_roll yaw(::vision_msgs::msg::PredictedArmor::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PredictedArmor_roll(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_pitch
{
public:
  explicit Init_PredictedArmor_pitch(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_yaw pitch(::vision_msgs::msg::PredictedArmor::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_PredictedArmor_yaw(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_z
{
public:
  explicit Init_PredictedArmor_z(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_pitch z(::vision_msgs::msg::PredictedArmor::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PredictedArmor_pitch(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_y
{
public:
  explicit Init_PredictedArmor_y(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_z y(::vision_msgs::msg::PredictedArmor::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PredictedArmor_z(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_x
{
public:
  explicit Init_PredictedArmor_x(::vision_msgs::msg::PredictedArmor & msg)
  : msg_(msg)
  {}
  Init_PredictedArmor_y x(::vision_msgs::msg::PredictedArmor::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PredictedArmor_y(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

class Init_PredictedArmor_header
{
public:
  Init_PredictedArmor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PredictedArmor_x header(::vision_msgs::msg::PredictedArmor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PredictedArmor_x(msg_);
  }

private:
  ::vision_msgs::msg::PredictedArmor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_msgs::msg::PredictedArmor>()
{
  return vision_msgs::msg::builder::Init_PredictedArmor_header();
}

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__BUILDER_HPP_
