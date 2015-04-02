# Copyright: (C) 2012 RobotCub Consortium
# Authors: Katrin Solveig Lohan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "objectAura")
PROJECT(${KEYWORD})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/anticipator/include
    ${PROJECT_SOURCE_DIR}/detectcollision/include
    ${PROJECT_SOURCE_DIR}/interface/include
    ${PROJECT_SOURCE_DIR}/objecthandler/include
	//${PROJECT_SOURCE_DIR}/auraCallculator/include
    ${YARP_INCLUDE_DIRS}
    ${ICUB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${iKin_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	anticipator/src/*.cpp
	detectcollision/src/*.cpp
	interface/src/*.cpp
	objecthandler/src/*.cpp
	auraCallculator/src/*.cpp
)
FILE(GLOB folder_header 
	anticipator/include/iCub/*.h
	detectcollision/include/iCub/*.h
	interface/include/iCub/*.h
	objecthandler/include/iCub/*.h
	auraCallculator/include/iCub/*.h
)
message("Source: ${folder_source}")
message("Header: ${folder_header}")
message("Sourcedir: ${PROJECT_SOURCE_DIR}")
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES )

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD} 
	        ${YARP_LIBRARIES}
	        ${OpenCV_LIBRARIES}
	        iKin
	        icubmod 
	)	

    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)





