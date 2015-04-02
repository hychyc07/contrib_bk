# Author: Alexandre Bernardino
# Copyright (C) 2008 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME camExtrinsicsCalib)
project(${PROJECTNAME})

find_package(YARP)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
find_package(OpenCV REQUIRED)

set(folder_source camExtrinsicsCalib.cpp)

source_group("Source Files" FILES ${folder_source})

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
