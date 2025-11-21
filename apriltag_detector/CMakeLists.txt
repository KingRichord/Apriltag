cmake_minimum_required(VERSION 3.0.2)
project(apriltag_detector)
message(STATUS Project_name:${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++14 support: OK")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
set(CMAKE_BUILD_TYPE "Release")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
#############################  编译设置  ################################

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")


add_subdirectory(mit)
add_subdirectory(umich)

#############################  ROS系统包  ################################
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  apriltag_msgs
)
###########################       外部包     ################################
# Eigen
find_package(Eigen3)
message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
#################################  OpenCV  ##################################
# Set OpenCV的基本路径
find_package(OpenCV QUIET)
#endif ()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

catkin_package()
###########################  头文件加载  ##################################
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
###########################  可执行文件生成  ################################
add_executable(${PROJECT_NAME}
        src/main.cpp
        )
###########################  链接库文件  ################################
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES}
        apriltag_mit
        apriltag_umich
)
