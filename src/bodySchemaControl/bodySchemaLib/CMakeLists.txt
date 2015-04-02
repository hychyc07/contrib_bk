# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Dalia De Santis, Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set (PROJECTNAME BodySchemaLib)
PROJECT(${PROJECTNAME})

SET (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Prefix directory for installing executables")
SET (BODYSCHEMALIB_BINARY_DIR "${CMAKE_SOURCE_DIR}/build")
message ("bodychemalib binary dir: ${BODYSCHEMALIB_BINARY_DIR}")

# create variables for installing the libraries:
# BODYSCHEMALIB_INSTALL_DIR: directory prefix for .lib objects
# BODYSCHEMALIB_INCLUDE_DIR: directory prefix for header files
# BODYSCHEMALIB_INCLUDES: will contain complete paths to all the .h files in the installed dir
# BODYSCHEMALIB_LIBS: will contain the relative path to all the .lib objects
# BODYSCHEMALIB_HEADERS: will contain complete paths to all the .h files
# BODYSCHEMALIB_LIB_NAMES: will contain all the .lib object names

SET( BODYSCHEMALIB_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib CACHE PATH "prefix for installing libraries")
SET( BODYSCHEMALIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/iCub CACHE PATH "prefix for installing headers")
SET( BODYSCHEMALIB_LIBS "" CACHE PATH "Relative paths to libraries")
SET( BODYSCHEMALIB_HEADERS "" CACHE PATH "Headers")

set(CMAKE_LIBRARY_PATH "${BODYSCHEMALIB_INSTALL_DIR}/${BODYSCHEMALIB_LIBS}")


# add lemon library:
add_subdirectory(mini_lemon)
add_subdirectory(pmp_lib)



#SET(BODYSCHEMALIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mini_lemon/include)

#IF(UNIX)
#	file (GLOB LEMON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/mini_lemon/lib/*.a)
#ELSE(UNIX)
#	file (GLOB LEMON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/mini_lemon/lib/*.lib)
#ENDIF(UNIX)
	
	
SET(BODYSCHEMALIB_NAMES ${LEMON_LIBRARY})
