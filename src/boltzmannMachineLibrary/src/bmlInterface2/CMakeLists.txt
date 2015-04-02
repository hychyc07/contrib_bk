# Copyright: (C) 2010 RobotCub Consortium
# Authors: Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)


SET(KEYWORD "bmlInterface")
PROJECT(${KEYWORD})


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
    ${BML_INCLUDE_DIRS}	
    ${GtkPlus_INCLUDE_DIRS}	
    ${OpenCV_INCLUDE_DIR}	
    ${IPP_INCLUDE_DIRS}	
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

    TARGET_LINK_LIBRARIES(${KEYWORD} ${BML_LIBRARIES}
				     icubmod  	
                                     ${YARP_LIBRARIES}
				     ${GtkPlus_LIBRARIES}		
				     ${OpenCV_LIBS}	
				     ${IPP_LIBRARIES}
    				     )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
