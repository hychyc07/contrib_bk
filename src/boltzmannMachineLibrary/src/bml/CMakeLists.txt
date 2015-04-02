# Copyright: (C) 2010 RobotCub Consortium
# Authors: Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME bml)

PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
		    ${ICUB_CONTRIB}/include	
)

INCLUDE_DIRECTORIES(include/iCub)
INCLUDE_DIRECTORIES(${ICUB_CONTRIB}/include)
INCLUDE_DIRECTORIES(.)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
#ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

# add functionalities like icub_add_library/icub_add_executable
#include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
MESSAGE(STATUS "icub_dir: ${ICUB_DIR}")
include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
                    
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
                                   DESTINATION include/iCub
                                   FILES ${folder_header})



