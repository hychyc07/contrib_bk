# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete and Simone Denei
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME skinCalibration)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
set (SKINCALIBRATION_SIMULATION OFF CACHE BOOL "Build the module for the simulation")
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)


if (SKINCALIBRATION_SIMULATION)
add_definitions(-DSKINCALIBRATION_SIMULATION)
endif (SKINCALIBRATION_SIMULATION)

# Add the application "skinCalibration"
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")

FILE(GLOB folder_source src/*.cpp )
FILE(GLOB folder_header include/iCub/skinCalibration/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/../skinForceControlLib/include
                    ${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     skinForceControlLib icubmod ctrlLib iKin)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

add_subdirectory(app)
