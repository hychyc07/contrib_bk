cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME controlBasis)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(LEARNINGMACHINE_DIR $ENV{ICUB_ROOT}/src/learningMachine)
SET(ICUB_CONTRIB $ENV{ICUB_ROOT}/contrib)

SET(CMAKE_MODULE_PATH ${YARP_DIR}/conf ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_DIR}/conf ${CMAKE_MODULE_PATH})


FILE(GLOB folder_source 
  resources/*.cpp
  potentialFunctions/*.cpp
  jacobians/*.cpp
  control/*.cpp
  util/statistics/*.cpp
  )
FILE(GLOB folder_header 
  include/*.h
  resources/*.h
  potentialFunctions/*.h
  jacobians/*.h
  control/*.h
  util/statistics/*.h
  )

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(PROJECT_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_SOURCE_DIR}/jacobians
  ${PROJECT_SOURCE_DIR}/potentialFunctions
  ${PROJECT_SOURCE_DIR}/control
  ${PROJECT_SOURCE_DIR}/util/statistics
)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_SOURCE_DIR}/jacobians
  ${PROJECT_SOURCE_DIR}/potentialFunctions
  ${PROJECT_SOURCE_DIR}/control
  ${PROJECT_SOURCE_DIR}/util/statistics
)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${rl_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${ICUB_CONTRIB}/include)
INCLUDE_DIRECTORIES(${LEARNINGMACHINE_DIR}/include)

#INCLUDE_DIRECTORIES(${ICUB_DIR}/../src/learningMachine)
#LINK_DIRECTORIES(${ICUB_DIR}/../src/learningMachine)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin) 
TARGET_LINK_LIBRARIES(${PROJECTNAME} rl) 
TARGET_LINK_LIBRARIES(${PROJECTNAME} learningMachine)

# add functionalities like icub_add_library/icub_add_executable
#include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
MESSAGE(STATUS "icub_dir: ${ICUB_DIR}")

icub_export_library(${PROJECTNAME}
  INTERNAL_INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS} 
  DESTINATION include/controlBasis
  FILES ${folder_header}
)



