# Copyright: (C) 2010 RobotCub Consortium
# Authors: Bjoern Browatzki
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# objSeg CMake file.
#
cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME objSeg)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
find_package(OpenCV)
find_package(Boost COMPONENTS system filesystem REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${iKin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c src/maxflow-v3.02/*.cpp)
file(GLOB folder_header include/iCub/*.h src/maxflow-v3.02/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} 
	${folder_source} 
	${folder_header}
)
 
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} icubmod) 

