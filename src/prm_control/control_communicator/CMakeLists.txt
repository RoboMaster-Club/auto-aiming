cmake_minimum_required(VERSION 3.5)
project(control_communicator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(/usr/include)
include_directories(include)

add_executable(ControlCommunicatorNode 
  src/ControlCommunicatorNode.cpp
  src/ControlCommunicator.cpp
  src/projectile_angle_convel.cpp
  src/utils.cpp
)


ament_target_dependencies(ControlCommunicatorNode 
  rclcpp
  std_msgs
  geometry_msgs
  vision_msgs
  tf2_ros
  tf2
  nav_msgs
)

target_link_libraries(ControlCommunicatorNode Eigen3::Eigen)

install(TARGETS
  ControlCommunicatorNode
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
