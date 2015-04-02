# Copyright: (C) 2012 RobotCub Consortium
# Authors: Christian Dondrup
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "SpeechSegmentSearch")
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
	    ${FFTW3_INCLUDE_DIRS}
	    ${MUSICRETR-1.0_INCLUDE_DIRS}
	    ${YARP_INCLUDE_DIRS}
	    ${ICUB_INCLUDE_DIRS}
	    ${iKin_INCLUDE_DIRS}
)

MACRO(CAR var)
  SET(${var} ${ARGV1})
ENDMACRO(CAR)

MACRO(CDR var junk)
  SET(${var} ${ARGN})
ENDMACRO(CDR)

MACRO(LIST_INDEX var index)
  SET(list . ${ARGN})
  FOREACH(i RANGE 1 ${index})
    CDR(list ${list})
  ENDFOREACH(i)
  CAR(${var} ${list})
ENDMACRO(LIST_INDEX)

LIST_INDEX(auxsrcdir 2 ${MUSICRETR-1.0_INCLUDE_DIRS})

aux_source_directory(${auxsrcdir} MUSICRETR-1.0_SOURCES)

# Set up the main executable.
IF (folder_source)
	    ADD_EXECUTABLE(${KEYWORD} 
			${folder_source} 
			${folder_header}
			${MUSICRETR-1.0_SOURCES}
	    )

		LINK_DIRECTORIES(
			${FFTW3_LIBRARY_DIRS}
			${MUSICRETR-1.0_LIBRARY_DIRS}
		)
		
	    TARGET_LINK_LIBRARIES(${KEYWORD} 
			${YARP_LIBRARIES}
			${FFTW3_LIBRARIES}
			${MUSICRETR-1.0_LIBRARIES}
			iKin
			icubmod 
		)	

    	INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    	MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)





