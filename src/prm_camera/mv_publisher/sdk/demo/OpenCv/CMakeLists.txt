cmake_minimum_required(VERSION 3.5)
project(MVOPENCV)

find_package(OpenCV)

include_directories(/lib)

add_executable(main main.cpp)

target_link_libraries(main ${OpenCV_LIBS} libMVSDK.so)
