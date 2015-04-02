# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea, Dalia De Santis
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "testThreadGroup")
PROJECT(${KEYWORD})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
	${PMPLIB_INCLUDE_DIR}
	${YARP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

message ( "pmp lib name: ${PMPLIB_LIBRARY}")
message ( "pmp lib path: ${PMPLIB_DIR}/${PMPLIB_LIBRARY}")

    TARGET_LINK_LIBRARIES(	${KEYWORD} 
				${YARP_LIBRARIES}
				pmp_lib)
							
    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)
