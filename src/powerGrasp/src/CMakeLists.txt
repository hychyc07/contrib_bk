# Copyright: 2010 RobotCub Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(PROJECTNAME powerGrasp)

project(${PROJECTNAME})

find_package(PCL 1.6 REQUIRED)

set (thrift3D_DIR ../thrift3D/build CACHE PATH "The directory containing a CMake configuration file for the thrift3D generated library")
find_package(thrift3D REQUIRED)

set (minimumBoundingBox_DIR ../minimumBoundingBox/build CACHE PATH "The directory containing a CMake configuration file for the minimumBoundingBox library")
find_package(minimumBoundingBox REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${thrift3D_INCLUDE_DIRS} ${minimumBoundingBox_INCLUDE_DIRS})

include(iCubOptions)
include(iCubHelpers)

set(header include/powerGrasp.h include/visualizationThread.h include/orientationThread.h)

if(NOT UNIX)
	set(header ${header}
		include/custom/dirent.h)
endif()
file(GLOB source src/*.cpp)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECTNAME} ${source} ${header})
target_link_libraries (${PROJECTNAME} ${PCL_LIBRARIES} ${YARP_LIBRARIES} minimumBoundingBox icubmod ctrlLib learningMachine thrift3D)
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
