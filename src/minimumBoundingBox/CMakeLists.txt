# Copyright: (C) 2011 RobotCub Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME minimumBoundingBox)

project(${PROJECTNAME})

find_package(YARP)
find_package(PCL 1.6 REQUIRED)

set (thrift3D_DIR ../thrift3D/build CACHE PATH "The directory containing a CMake configuration file for the thrift3D generated library")
find_package(thrift3D REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${thrift3D_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(header_private include/iCub/data3D/private/helpers.h)

file(GLOB header include/iCub/data3D/*.h)
file(GLOB source src/*.cpp)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header} ${header_private})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECTNAME} ${source} ${header} ${header_private})
target_link_libraries(${PROJECTNAME} ${PCL_LIBRARIES} ${YARP_LIBRARIES} thrift3D)

export(TARGETS ${PROJECTNAME} FILE ${PROJECTNAME}.cmake)

install(TARGETS ${PROJECTNAME} EXPORT ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/lib)
install(EXPORT ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/lib/${PROJECTNAME})

# export library
export(PACKAGE ${PROJECTNAME})
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
configure_file(libraryConfig.cmake.in ${CMAKE_BINARY_DIR}/${PROJECTNAME}Config.cmake @ONLY)
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
set(${PROJECTNAME}_INCLUDE_DIRS ${inc_dirs} CACHE INTERNAL "MinimumBoundingBox library's include directory")
