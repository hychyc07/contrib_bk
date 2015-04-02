cmake_minimum_required (VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(YARP)
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rospack(ikart)
#genmsg()

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_executable(iKartRosBridge main.cpp bridgeThread.cpp)



TARGET_LINK_LIBRARIES(iKartRosBridge ${YARP_LIBRARIES})



