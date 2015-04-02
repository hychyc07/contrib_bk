#
# Copyright (C) 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME salience)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

find_package(OpenCV)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include ${iCubVis_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} iCubVis ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

icub_app(${PROJECTNAME})
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
icub_app_install(${PROJECTNAME}
                FILES ${conf}
				DESTINATION conf)
                
                