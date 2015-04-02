# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(projectname ScSPMClassifier)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
INCLUDE(iCubOptions)
include(iCubHelpers)

find_package(OpenCV)

cmake_minimum_required(VERSION 2.6)

project(${projectname})
file(GLOB source *.cpp)
file(GLOB header *.h)
source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
   
add_executable(${projectname} ${source} ${header} )
target_link_libraries(${projectname} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} )
install(TARGETS ${projectname} DESTINATION ${ICUB_DIR}/bin)

