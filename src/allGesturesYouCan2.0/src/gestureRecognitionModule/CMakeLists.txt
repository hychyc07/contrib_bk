# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

project(gestureRecognitionModule)

find_package(OpenCV)
find_package(stereoVision)

include(iCubOptions)
include(iCubHelpers)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${stereoVision_INCLUDE_DIRS})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(gestureRecognitionModule ${source} ${header} )
target_link_libraries(gestureRecognitionModule ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} stereoVision ctrlLib)
install(TARGETS gestureRecognitionModule DESTINATION ${ICUB_DIR}/bin)

