# Copyright (C) 2011 RBCS IIT
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME learningEndEffectorWrench)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

# Add the application "learningEndEffectorWrenchDemo"
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
add_subdirectory(learningEndEffectorWrenchDemo)
icub_app_all()

FILE(GLOB folder_source src/*.cpp )
FILE(GLOB folder_header include/iCub/learningEndEffectorWrench/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     ${YARP_LIBRARIES}
                                     icubmod 
                                     learningMachine)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
