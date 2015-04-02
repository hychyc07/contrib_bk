# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME Grasper_DARWIN)
PROJECT(${PROJECTNAME})

SET(folder_header 	include/darwin/Grasp.h)

SET(folder_source 	src/Grasp.cpp
					src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
					${YARP_INCLUDE_DIRS})
		
ADD_EXECUTABLE(${PROJECTNAME}  ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}  	${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
