# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

project(gestureRecognitionModule)

find_package(OpenCV)
include(FindKinectSDK.cmake)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${KinectSDK_INCLUDE_DIR})

LINK_DIRECTORIES(${KinectSDK_LIB_DIR})
file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(gestureRecognitionModule ${source} ${header} )
target_link_libraries(gestureRecognitionModule ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${KinectSDK_LIBRARIES} ctrlLib)
install(TARGETS gestureRecognitionModule DESTINATION ${ICUB_DIR}/bin)

