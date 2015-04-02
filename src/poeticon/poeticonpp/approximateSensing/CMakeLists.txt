# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME approximateSensing)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(YARP)

FIND_PACKAGE(ICUB)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
INCLUDE(iCubOptions)
include(iCubHelpers)

FIND_PACKAGE(OpenCV)

include_directories(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS} 
		     ${OPENCV_INCLUDE_DIR})

file(GLOB folder_source *.cpp)
file(GLOB_RECURSE folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} icubmod)

install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
#add_subdirectory(app)

