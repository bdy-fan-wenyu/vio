cmake_minimum_required(VERSION 3.10 )
project(camera)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
aux_source_directory(src camera_SRC_DIR)
add_library(camera ${camera_SRC_DIR})
target_include_directories(camera PUBLIC include)
target_link_libraries(camera ${OpenCV_LIBS})