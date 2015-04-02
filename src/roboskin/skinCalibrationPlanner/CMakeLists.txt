# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME skinCalibrationPlanner)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)
#### assume FindACE.cmake is in one of the CMAKE_MODULE_PATH folders.
FIND_PACKAGE(ACE)


FILE(GLOB folder_source src/*.cpp )
FILE(GLOB folder_header include/iCub/skinCalibrationPlanner/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${skinDynLib_INCLUDE_DIRS}
					${iDyn_INCLUDE_DIRS}
					${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/../skinForceControlLib/include
					${ACE_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} skinForceControlLib
                                     iDyn
                                     ${YARP_LIBRARIES}
                                     skinDynLib
                                     icubmod 
									 ${ACE_LIBRARIES})
#MESSAGE( ${CMAKE_BUILD_TYPE} )
#if( CMAKE_BUILD_TYPE MATCHES Release)
 #  TARGET_LINK_LIBRARIES(${PROJECTNAME} 
  #  ${PROJECT_SOURCE_DIR}/../skinForceControl/build/Release/skinForceControlLib.lib )
#else()
 #   TARGET_LINK_LIBRARIES(${PROJECTNAME} 
  #  ${PROJECT_SOURCE_DIR}/../skinForceControl/build/Debug/skinForceControlLibd.lib )
#endif()                                    

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
