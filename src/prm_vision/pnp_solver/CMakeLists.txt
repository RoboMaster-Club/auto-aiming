cmake_minimum_required(VERSION 3.5)
project(pnp_solver)


include_directories(/usr/include)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(KalmanFilter STATIC src/KalmanFilter.cpp)
target_link_libraries(KalmanFilter Eigen3::Eigen)

add_library(ValidityFilter STATIC src/ValidityFilter.cpp)

add_executable(PNPSolverNode
  src/PNPSolver.cpp
  src/projectile_angle_convel.cpp
)
target_link_libraries(PNPSolverNode KalmanFilter ValidityFilter)
ament_target_dependencies(PNPSolverNode 
  rclcpp
  vision_msgs
  geometry_msgs
  tf2
  tf2_ros
  OpenCV
)

install(
  TARGETS PNPSolverNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp vision_msgs geometry_msgs tf2)

ament_package()

