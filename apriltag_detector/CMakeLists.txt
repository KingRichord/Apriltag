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

#############################  OPENMP加速  ################################
#find_package(OpenMP REQUIRED)
#if(OPENMP_FOUND)
# message("OPENMP FOUND")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()
#############################  ROS系统包  ################################
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  dbrobot_msg
  tf2_geometry_msgs
  tf2_ros
  apriltag_mit
  apriltag_umich
  apriltag_msgs
)
###########################       外部包     ################################
# Eigen
find_package(Eigen3)
message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
#################################  OpenCV  ##################################
# Set OpenCV的基本路径
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV QUIET)
#set(OpenCV_LIB_DIR "/home/moi/include/opencv4.5/lib/cmake/opencv4/" CACHE STRING "The directory containing a CMake configuration file for OpenCV.")
#message(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
#find_package(OpenCV 4.5 QUIET
#        COMPONENTS
#        core
#        imgproc
#        highgui
#        )
#if (NOT OpenCV_FOUND)
# find_package(OpenCV  PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH REQUIRED
#         COMPONENTS
#         core
#         imgproc
#         highgui
#         )
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
        ${OpenCV_LIBRARIES})
