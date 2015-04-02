# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME rl)
PROJECT(${PROJECTNAME})

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.
FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

#SET(CMAKE_MODULE_PATH $ENV{ICUB_ROOT}/main/conf ${CMAKE_MODULE_PATH})
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


include(iCubOptions)
include(iCubHelpers)

FILE(GLOB folder_source 
  MarkovDecisionProcess.cpp
  tinyxml/*.cpp
  )
FILE(GLOB folder_header 
  MarkovDecisionProcess.h
  tinyxml/*.h
  )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(tinyxml ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES})

# add functionalities like icub_add_library/icub_add_executable
include($ENV{ICUB_ROOT}/main/conf/iCubHelpers.cmake)

MESSAGE(STATUS "INSTALL: ${CMAKE_INSTALL_PREFIX}")
icub_export_library(${PROJECTNAME}
  INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} tinyxml
  DESTINATION include
  FILES ${folder_header}
)
