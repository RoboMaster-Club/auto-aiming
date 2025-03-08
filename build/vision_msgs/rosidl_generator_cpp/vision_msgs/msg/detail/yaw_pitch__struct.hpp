// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/YawPitch.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__YAW_PITCH__STRUCT_HPP_
#define VISION_MSGS__MSG__DETAIL__YAW_PITCH__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__YawPitch __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__YawPitch __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YawPitch_
{
  using Type = YawPitch_<ContainerAllocator>;

  explicit YawPitch_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->dst = 0.0;
    }
  }

  explicit YawPitch_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->dst = 0.0;
    }
  }

  // field types and members
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _dst_type =
    double;
  _dst_type dst;

  // setters for named parameter idiom
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__dst(
    const double & _arg)
  {
    this->dst = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::YawPitch_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::YawPitch_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::YawPitch_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::YawPitch_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__YawPitch
    std::shared_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__YawPitch
    std::shared_ptr<vision_msgs::msg::YawPitch_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YawPitch_ & other) const
  {
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->dst != other.dst) {
      return false;
    }
    return true;
  }
  bool operator!=(const YawPitch_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YawPitch_

// alias to use template instance with default allocator
using YawPitch =
  vision_msgs::msg::YawPitch_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__YAW_PITCH__STRUCT_HPP_
