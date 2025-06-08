// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_
#define VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__KeyPoints __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__KeyPoints __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyPoints_
{
  using Type = KeyPoints_<ContainerAllocator>;

  explicit KeyPoints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 8>::iterator, float>(this->points.begin(), this->points.end(), 0.0f);
      this->is_large_armor = false;
    }
  }

  explicit KeyPoints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    points(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 8>::iterator, float>(this->points.begin(), this->points.end(), 0.0f);
      this->is_large_armor = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::array<float, 8>;
  _points_type points;
  using _is_large_armor_type =
    bool;
  _is_large_armor_type is_large_armor;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::array<float, 8> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__is_large_armor(
    const bool & _arg)
  {
    this->is_large_armor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::KeyPoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::KeyPoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::KeyPoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::KeyPoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__KeyPoints
    std::shared_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__KeyPoints
    std::shared_ptr<vision_msgs::msg::KeyPoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyPoints_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->is_large_armor != other.is_large_armor) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyPoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyPoints_

// alias to use template instance with default allocator
using KeyPoints =
  vision_msgs::msg::KeyPoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_
