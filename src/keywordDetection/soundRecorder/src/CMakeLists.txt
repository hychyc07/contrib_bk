# Copyright: (C) 2012 RobotCub Consortium
# Authors: Christian Dondrup
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "SoundRecorder")
PROJECT(${KEYWORD})

# Search for source code.
FILE(GLOB_RECURSE folder_source *.cpp)
FILE(GLOB_RECURSE folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#Search for include directories
set (MY_INCLUDE_DIRS "")
foreach (_headerFile ${folder_header})
    	get_filename_component(_dir ${_headerFile} PATH)
	get_filename_component(_dir ${_dir} PATH)
	list (APPEND MY_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES MY_INCLUDE_DIRS)

INCLUDE_DIRECTORIES(
	${MY_INCLUDE_DIRS}
	${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	${iKin_INCLUDE_DIRS}
	${SNDFILE_INCLUDE_DIRS}
	${PORTAUDIO_INCLUDE_DIRS}
)

# Set up the main executable.
IF (folder_source)
	ADD_EXECUTABLE(${KEYWORD} 
		${folder_source} 
		${folder_header}
	)

	LINK_DIRECTORIES(
		${SNDFILE_LIBRARY_DIRS}
	)
		
	TARGET_LINK_LIBRARIES(${KEYWORD} 
		${YARP_LIBRARIES}
		${SNDFILE_LIBRARIES}
		${PORTAUDIO_LIBRARIES}
		iKin
		icubmod 
	)	

    	INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    	MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)





