# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME pmp_lib)
PROJECT(${PROJECTNAME})

#set (PROJECT_BINARY_DIR ${BODYSCHEMALIB_BINARY_DIR}/pmp_lib CACHE STRING "pmp_lib project binary dir")

# FIND DEPENDENCIES:
# --------------------------------------------------------------------------------------------------

#FIND_PACKAGE(YARP REQUIRED)
#FIND_PACKAGE(ICUB REQUIRED)
#FIND_PACKAGE(GSL)
	
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/find_gsl.cmake)

		
# --------------------------------------------------------------------------------------------------

#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)

SET(folder_source 	src/core/pmpGenericBlock_if.cpp
					src/core/pmpGenericBlock.cpp
					src/core/tbg.cpp
					src/core/VTGS.cpp
					src/groupThread.cpp
					)

SET(folder_header 	include/pmp_lib/core/pmpGenericBlock_if.h
					include/pmp_lib/core/pmpGenericBlock.h
					include/pmp_lib/core/tbg.h
					include/pmp_lib/core/VTGS.h
					include/pmp_lib/pmp_interface.h
					include/pmp_lib/pmp_utils.h
					include/pmp_lib/groupThread.h
					)
					
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

message (STATUS "${PROJECT_SOURCE_DIR}/include")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
		    ${iKin_INCLUDE_DIRS}
		    ${LEMON_INCLUDE_DIR})
		    #${EXTERNAL_INCLUDES}	)

#IF(UNIX)
#  SET_TARGET_PROPERTIES(pmp_lib PROPERTIES OUTPUT_NAME pmp)
#ENDIF(UNIX)
  
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} 	iKin
					${YARP_LIBRARIES}
                                        ${ICUB_LIBRARIES}
				#	${EXTERNAL_LIBRARIES}
					lemon)

set(BODYSCHEMALIB_LIBS ${BODYSCHEMALIB_LIBS}/pmp_lib)

#define here find package variables: PMPLIB_INCLUDE_DIR, PMPLIB_DIR, PMPLIB_LIBRARY
set(PMPLIB_INCLUDE_DIR 	${PROJECT_SOURCE_DIR}/include
			CACHE INTERNAL "Path to pmp_lib header files")
message ( "Pmp lib include dir: ${PMPLIB_INCLUDE_DIR}")
						

IF(UNIX)	
  set(PMPLIB_LIBRARY libpmp_lib CACHE INTERNAL "pmp_lib name")
  set(PMPLIB_DIR ${PROJECT_BINARY_DIR} CACHE INTERNAL "Path to pmp_lib object")
ELSE(UNIX)
	set(PMPLIB_LIBRARY pmp_lib CACHE INTERNAL "pmp_lib name")
	set(PMPLIB_DIR	${PROJECT_BINARY_DIR}/pmp_lib/Debug 
	  ${PROJECT_BINARY_DIR}/pmp_lib/Release
		CACHE INTERNAL "Path to pmp_lib object")
ENDIF(UNIX)
set(BODYSCHEMALIB_LIB_NAMES ${PMPLIB_LIBRARY})

message( "pmp lib dir is: ${PMPLIB_DIR}")

INSTALL(
  TARGETS pmp_lib
  ARCHIVE DESTINATION ${BODYSCHEMALIB_INSTALL_DIR}/pmp_lib
  COMPONENT library
)

INSTALL(
  DIRECTORY include/pmp_lib
  DESTINATION ${BODYSCHEMALIB_INCLUDE_DIR}
  COMPONENT headers
  FILES_MATCHING PATTERN "*.h"
)