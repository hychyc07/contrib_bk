# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(LIBNAME skinForceControlLib)
PROJECT(${LIBNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)
#### assume FindACE.cmake is in one of the CMAKE_MODULE_PATH folders.
FIND_PACKAGE(ACE)


FILE(GLOB source_lib    src/skinForceControlClient.cpp
                        src/skinManagerClient.cpp
                        src/skinCalibrationClient.cpp
                        src/controlLaws.cpp
                        src/robot_interfaces.cpp
                        src/robotArm.cpp
                        src/util.cpp 
                        src/utilRobotics.cpp )
FILE(GLOB header_lib    include/iCub/skinForceControl/skinForceControlClient.h
                        include/iCub/skinForceControl/skinManagerClient.h
                        include/iCub/skinForceControl/skinCalibrationClient.h
                        include/iCub/skinForceControl/controlLaws.h
                        include/iCub/skinForceControl/robot_interfaces.h
                        include/iCub/skinForceControl/robotArm.h
                        include/iCub/skinForceControl/util.h 
                        include/iCub/skinForceControl/utilRobotics.h 
                        include/iCub/skinForceControl/skinForceControlLib.h)


INCLUDE_DIRECTORIES(${skinDynLib_INCLUDE_DIRS}
					${iDyn_INCLUDE_DIRS}
					${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/
					${ACE_INCLUDE_DIRS})

add_library(${LIBNAME} ${source_lib} ${header_lib})


