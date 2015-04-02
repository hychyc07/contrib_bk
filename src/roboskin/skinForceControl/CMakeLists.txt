# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME skinForceControl)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)
#### assume FindACE.cmake is in one of the CMAKE_MODULE_PATH folders.
FIND_PACKAGE(ACE)


# Add the application "skinForceControl"
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
add_subdirectory(skinForceControlDemo)

FILE(GLOB folder_source src/main.cpp 
                        src/controlThread.cpp
                        src/controlPlanner.cpp
                        src/actions.cpp )
FILE(GLOB folder_header include/iCub/skinForceControl/controlThread.h
                        include/iCub/skinForceControl/controlPlanner.h
                        include/iCub/skinForceControl/actions.h 
                        include/iCub/skinForceControl/controlConstants.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/../skinForceControlLib/include
                    ${skinDynLib_INCLUDE_DIRS}
					${iDyn_INCLUDE_DIRS}
					${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
					${ACE_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${LIBNAME}
                                     iDyn
                                     ${YARP_LIBRARIES}
                                     skinDynLib
                                     icubmod 
									 ${ACE_LIBRARIES}
                                     skinForceControlLib )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
