cmake_minimum_required(VERSION 3.5)
project(apriltag_mit)

set(USE_ROS TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
