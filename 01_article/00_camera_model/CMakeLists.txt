cmake_minimum_required(VERSION 3.0)

project(pj_cv_camera_model)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

add_executable(pj_cv_camera_model main.cpp camera_model.h)

