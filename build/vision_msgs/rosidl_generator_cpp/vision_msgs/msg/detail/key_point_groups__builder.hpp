// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_msgs:msg/KeyPointGroups.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__BUILDER_HPP_
#define VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__BUILDER_HPP_

#include "vision_msgs/msg/detail/key_point_groups__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vision_msgs
{

namespace msg
{

namespace builder
{

class Init_KeyPointGroups_groups
{
public:
  Init_KeyPointGroups_groups()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vision_msgs::msg::KeyPointGroups groups(::vision_msgs::msg::KeyPointGroups::_groups_type arg)
  {
    msg_.groups = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_msgs::msg::KeyPointGroups msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_msgs::msg::KeyPointGroups>()
{
  return vision_msgs::msg::builder::Init_KeyPointGroups_groups();
}

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__BUILDER_HPP_
