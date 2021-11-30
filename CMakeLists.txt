cmake_minimum_required(VERSION 3.0)

project(opencv_sample)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

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
add_subdirectory(dnn_depth_midas)
add_subdirectory(reconstruction_depth_to_3d)
