# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME VergencePointing)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source 	src/Demo1Main.cpp
					src/Demo1Module.cpp
					src/Demo1Helper.cpp)

SET(folder_header 	include/darwin/Demo1Helper.h
					include/darwin/Demo1Module.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 	${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin )