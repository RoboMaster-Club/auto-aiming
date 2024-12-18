cmake_minimum_required(VERSION 3.8)
project(opencv_armor_detector CXX CUDA)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)  # GTest

# Include directories
include_directories(/usr/include)
include_directories(include)

# Build target
add_library(OpenCVArmorDetector STATIC src/OpenCVArmorDetector.cpp)
target_link_libraries(OpenCVArmorDetector ${OpenCV_LIBS})

add_executable(OpenCVArmorDetectorNode src/OpenCVArmorDetectorNode.cpp)
target_link_libraries(OpenCVArmorDetectorNode OpenCVArmorDetector ${OpenCV_LIBS})

ament_target_dependencies(OpenCVArmorDetectorNode 
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  vision_msgs
)

ament_target_dependencies(OpenCVArmorDetector
  rclcpp
)

# GTest setup
ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
target_include_directories(test_${PROJECT_NAME} PRIVATE include)
target_link_libraries(test_${PROJECT_NAME} OpenCVArmorDetector)

# Install the node
install(TARGETS OpenCVArmorDetectorNode
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs)
ament_package()
