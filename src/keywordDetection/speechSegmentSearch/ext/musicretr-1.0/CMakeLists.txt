# Copyright: (C) 2010 RobotCub Consortium
# Authors: Christian Dondrup
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(KEYWORD "musicretr-1.0")
PROJECT(${KEYWORD})
SET(TAR "tar.gz")
SET(downloaddir "downloads")
SET(creatlibdir "create_lib")

message(STATUS "Creating directories")
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E make_directory ${downloaddir})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}.${TAR}
	COMMAND wget -Nnv http://www.cs.cmu.edu/~yke/musicretrieval/${KEYWORD}.${TAR}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}
	COMMENT "Downloading sources"
	VERBATIM
)
add_custom_target(download ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}.${TAR})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}
	COMMAND tar xfz ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}.${TAR} --overwrite
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}.${TAR}
	COMMENT "Unpacking tar.gz archive"
	VERBATIM
)
add_custom_target(untar ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD})


add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}/${KEYWORD}.patch
	COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_CURRENT_SOURCE_DIR}/${KEYWORD}.patch
		${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}/	
	COMMAND patch -i ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}/${KEYWORD}.patch
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${KEYWORD}.patch ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}
	COMMENT "Patching sources"
	VERBATIM
)
add_custom_target(patch ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${downloaddir}/${KEYWORD}/${KEYWORD}.patch)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt.lib.in ${CMAKE_CURRENT_BINARY_DIR}/${creatlibdir}/CMakeLists.txt @ONLY)
