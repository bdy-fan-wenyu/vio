cmake_minimum_required(VERSION 3.10)
project(feature_tracker)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

aux_source_directory(src feature_tracker_SRC_DIRS)
add_library(feature_tracker ${feature_tracker_SRC_DIRS})
target_include_directories(feature_tracker PUBLIC include )
target_link_libraries(feature_tracker ${OpenCV_LIBS} camera imu)