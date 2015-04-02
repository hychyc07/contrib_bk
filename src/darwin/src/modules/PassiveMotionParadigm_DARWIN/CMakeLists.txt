# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME PassiveMotionParadigm_DARWIN)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

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
					${iKin_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 	iKin
										${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin )