# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME PMP_undistorted)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

#find GSL

set(PMP_HAS_GSL_LIB FALSE)

find_package(GSL REQUIRED)
  if (NOT GSL_FOUND)
  	message(FATAL_ERROR "GSL not found, won't compile PMP_undistorted module")
  else (NOT GSL_FOUND)
  	set(PMP_HAS_GSL_LIB TRUE)
	message (STATUS "found GSL: $ENV{GSL_DIR}")
  endif (NOT GSL_FOUND)
	
#if(PMP_HAS_GSL_LIB)

## find GSL libraries
#	set(GSL_LIB_PATH "$ENV{GSL_DIR}/lib" CACHE PATH "GSL_LIBRARIES path")
#	if (NOT IS_DIRECTORY ${GSL_LIB_PATH})
#		message ( FATAL ERROR "SPECIFY GSL libraries PATH")
#	endif (NOT IS_DIRECTORY ${GSL_LIB_PATH})

#	file (GLOB GSL_LIBRARIES ${GSL_LIB_PATH}/*.lib)
#	message (STATUS "found GSL libraries: ${GSL_LIBRARIES}")

## find GSL headers
#	set (GSL_INCLUDE_DIRS "$ENV{GSL_DIR}/include" CACHE PATH "GSL_INCLUDE_DIRS path")
#	if (NOT IS_DIRECTORY ${GSL_INCLUDE_DIRS})
#		message ( FATAL ERROR "SPECIFY GSL headers PATH")
#	endif (NOT IS_DIRECTORY ${GSL_INCLUDE_DIRS})

#endif(PMP_HAS_GSL_LIB)

SET(folder_source 	src/PMP.cpp
					src/PMPmain.cpp
					src/PMPmodule.cpp
					src/PMPthread.cpp
					src/VTGS.cpp	
					src/tbg.cpp       )

SET(folder_header 	include/darwin/PMP.h
					include/darwin/PMPmodule.h
					include/darwin/PMPthread.h
					include/darwin/VTGS.h
					include/darwin/tbg.h     )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
					${iKin_INCLUDE_DIRS}
					${GSL_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 	iKin
										${YARP_LIBRARIES}
										${GSL_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin )