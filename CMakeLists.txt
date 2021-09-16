cmake_minimum_required(VERSION 3.0)

project(opencv_sample)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

include_directories(cvui)
include_directories(common)
add_subdirectory(common)

add_subdirectory(undistortion_calibration)
add_subdirectory(undistortion_manual_unified_projection)
add_subdirectory(projection_points_3d_to_2d)
add_subdirectory(projection_image_3d_to_2d)
add_subdirectory(transformation_topview_projection)
add_subdirectory(transformation_homography)
add_subdirectory(distance_calculation)
add_subdirectory(curve_fitting)
add_subdirectory(dnn_face)
