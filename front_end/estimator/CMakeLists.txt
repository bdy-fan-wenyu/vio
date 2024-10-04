cmake_minimum_required(VERSION 3.10)
project(estimator)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
aux_source_directory(src estimator_SRC_DIR)
add_library(estimator ${estimator_SRC_DIR})
target_include_directories(estimator PUBLIC include)
target_link_libraries(estimator ${OpenCV_LIBS} imu feature_tracker feature)