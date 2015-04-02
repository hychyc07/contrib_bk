# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri, Tomas Werner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME ColorSegmentation_DARWIN)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source 	src/colorsegMain.cpp
					src/colorsegModule.cpp
					src/colorsegThread.cpp
					src/segment.cpp)

SET(folder_header 	include/darwin/colorsegModule.h
					include/darwin/colorsegThread.h
					include/darwin/segment.h
					include/darwin/disjoint-set.h
					include/darwin/fileutil.h     )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 	${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin )