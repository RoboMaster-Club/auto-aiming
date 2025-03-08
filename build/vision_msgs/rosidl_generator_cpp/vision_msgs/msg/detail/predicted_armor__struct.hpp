// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/PredictedArmor.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_HPP_
#define VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__PredictedArmor __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__PredictedArmor __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PredictedArmor_
{
  using Type = PredictedArmor_<ContainerAllocator>;

  explicit PredictedArmor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->roll = 0.0;
      this->x_vel = 0.0;
      this->y_vel = 0.0;
      this->z_vel = 0.0;
      this->fire = false;
    }
  }

  explicit PredictedArmor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->roll = 0.0;
      this->x_vel = 0.0;
      this->y_vel = 0.0;
      this->z_vel = 0.0;
      this->fire = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _roll_type =
    double;
  _roll_type roll;
  using _x_vel_type =
    double;
  _x_vel_type x_vel;
  using _y_vel_type =
    double;
  _y_vel_type y_vel;
  using _z_vel_type =
    double;
  _z_vel_type z_vel;
  using _fire_type =
    bool;
  _fire_type fire;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
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
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__x_vel(
    const double & _arg)
  {
    this->x_vel = _arg;
    return *this;
  }
  Type & set__y_vel(
    const double & _arg)
  {
    this->y_vel = _arg;
    return *this;
  }
  Type & set__z_vel(
    const double & _arg)
  {
    this->z_vel = _arg;
    return *this;
  }
  Type & set__fire(
    const bool & _arg)
  {
    this->fire = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::PredictedArmor_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::PredictedArmor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::PredictedArmor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::PredictedArmor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__PredictedArmor
    std::shared_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__PredictedArmor
    std::shared_ptr<vision_msgs::msg::PredictedArmor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PredictedArmor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->x_vel != other.x_vel) {
      return false;
    }
    if (this->y_vel != other.y_vel) {
      return false;
    }
    if (this->z_vel != other.z_vel) {
      return false;
    }
    if (this->fire != other.fire) {
      return false;
    }
    return true;
  }
  bool operator!=(const PredictedArmor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PredictedArmor_

// alias to use template instance with default allocator
using PredictedArmor =
  vision_msgs::msg::PredictedArmor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__PREDICTED_ARMOR__STRUCT_HPP_
