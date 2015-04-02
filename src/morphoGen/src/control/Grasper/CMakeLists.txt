# Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
SET(PROJECTNAME GrasperTest_DARWIN)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

#SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Grasper install prefix" FORCE)
#FIND_PACKAGE(YARP)

FILE(GLOB folder_header include/*.h)
		  
FILE(GLOB folder_source src/*.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
		    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME}  ${folder_source} ${folder_header})

#INSTALL_TARGETS (/build/bin ${PROJECTNAME})

TARGET_LINK_LIBRARIES(${PROJECTNAME}  ${YARP_LIBRARIES})
#                                     ${OpenCV_LIBRARIES}


#INSTALL(TARGETS ${PROJECTNAME} DESTINATION "C:/Programs/Development/darwin/trunk/kcl/unit testing/grasper/build")
INSTALL_TARGETS(/bin ${PROJECTNAME})