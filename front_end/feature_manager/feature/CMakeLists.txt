cmake_minimum_required(VERSION 3.10)
project(feature)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

aux_source_directory(src feature_SRC_DIRS)
add_library(feature ${feature_SRC_DIRS})
target_include_directories(feature PUBLIC include )
target_link_libraries(feature ${OpenCV_LIBS} camera imu)