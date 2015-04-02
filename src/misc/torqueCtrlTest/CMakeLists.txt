# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME torqueCtrlTest)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)


FILE(GLOB folder_source src/*.cpp )
FILE(GLOB folder_header include/iCub/torqueCtrlTest/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${ctrlLib_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
                                     iDyn
                                     ${YARP_LIBRARIES}
                                     skinDynLib
                                     icubmod )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
