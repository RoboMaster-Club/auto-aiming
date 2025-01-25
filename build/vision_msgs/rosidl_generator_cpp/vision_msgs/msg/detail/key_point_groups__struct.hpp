// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/KeyPointGroups.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_HPP_
#define VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'groups'
#include "vision_msgs/msg/detail/key_points__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__KeyPointGroups __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__KeyPointGroups __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyPointGroups_
{
  using Type = KeyPointGroups_<ContainerAllocator>;

  explicit KeyPointGroups_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->groups.fill(vision_msgs::msg::KeyPoints_<ContainerAllocator>{_init});
    }
  }

  explicit KeyPointGroups_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : groups(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->groups.fill(vision_msgs::msg::KeyPoints_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _groups_type =
    std::array<vision_msgs::msg::KeyPoints_<ContainerAllocator>, 2>;
  _groups_type groups;

  // setters for named parameter idiom
  Type & set__groups(
    const std::array<vision_msgs::msg::KeyPoints_<ContainerAllocator>, 2> & _arg)
  {
    this->groups = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::KeyPointGroups_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::KeyPointGroups_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::KeyPointGroups_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::KeyPointGroups_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__KeyPointGroups
    std::shared_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__KeyPointGroups
    std::shared_ptr<vision_msgs::msg::KeyPointGroups_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyPointGroups_ & other) const
  {
    if (this->groups != other.groups) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyPointGroups_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyPointGroups_

// alias to use template instance with default allocator
using KeyPointGroups =
  vision_msgs::msg::KeyPointGroups_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINT_GROUPS__STRUCT_HPP_
