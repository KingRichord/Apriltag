cmake_minimum_required(VERSION 3.5)
project(apriltag_umich)

set(USE_ROS TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
include_directories(include src)


file(GLOB C_FILES ${PROJECT_SOURCE_DIR}/src/*.c
          ${PROJECT_SOURCE_DIR}/src/common/*.c)
add_library(${PROJECT_NAME} SHARED ${C_FILES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})