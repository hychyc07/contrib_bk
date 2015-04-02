# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME eye2world)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

file(GLOB_RECURSE sources "src/*.cpp")
file(GLOB_RECURSE headers "include/iCub/*.h")

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} iKin ${YARP_LIBRARIES})
#install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

