# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME pmp_network_server)
PROJECT(${PROJECTNAME})

# FIND DEPENDENCIES:
# --------------------------------------------------------------------------------------------------

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

#lemon e pmp_lib

message(status "including dirs: ${PMPLIB_INCLUDE_DIR}, ${LEMON_INCLUDE_DIR}")

		
# --------------------------------------------------------------------------------------------------

SET(folder_source 	src/pmp_network_server.cpp
					src/pmp_network.cpp
					src/pmp_runner.cpp
					src/main.cpp
					)

SET(folder_header 	include/pmp_network_server.h
					include/pmp_network.h
					include/pmp_runner.h
					)
					
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include  ${YARP_INCLUDE_DIRS}
					${iKin_INCLUDE_DIRS}
					${PMPLIB_INCLUDE_DIR}
					${LEMON_INCLUDE_DIR}	)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} ${OTHERS_SRC})

#LINK_DIRECTORIES(${EXTERNAL_DIR})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 	iKin
					  ${YARP_LIBRARIES}
					  lemon
					  pmp_lib)								

#INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${PROJECT_SOURCE_DIR}/bin )
INSTALL_TARGETS(/bin ${PROJECTNAME})
