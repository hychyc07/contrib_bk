# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME LowLevelController_DARWIN)
PROJECT(${PROJECTNAME})

SET(folder_header 	include/darwin/DevDriverModule.h
					include/darwin/MessageDriver.h)

SET(folder_source 	src/DevDriverModule.cpp
					src/MessageDriver.cpp
					src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
					${YARP_INCLUDE_DIRS}
					${iKin_INCLUDE_DIRS})
		
ADD_LIBRARIES(${PROJECTNAME}  ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}  #VLFEAT lapack)
										
# to make libraries private
#INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)