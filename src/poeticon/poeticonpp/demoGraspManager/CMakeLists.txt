# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME graspManager)
PROJECT(${PROJECTNAME})
find_package(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
INCLUDE(iCubOptions)
include(iCubHelpers)

FIND_PACKAGE(OpenCV)

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${ctrlLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
                                     icubmod
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

