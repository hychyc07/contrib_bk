SET(KEYWORD "ARTrackerUH")
cmake_minimum_required(VERSION 2.6)

# Start a project.
PROJECT(${KEYWORD})  

# Find YARP.  Point the YARP_DIR environment variable at your build.

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(ARToolKit REQUIRED)
ADD_DEFINITIONS(-Wall -g)
# Search for source code.
FILE(GLOB folder_source ./src/*.cpp ./src/*.cc ./src/*.c /usr/lib/*.c)
FILE(GLOB folder_header ./include/iCub/*.h /usr/include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})  

# Automatically add include directories if needed. 
FOREACH(header_file ${folder_header})   
	GET_FILENAME_COMPONENT(p ${header_file} PATH)   
	INCLUDE_DIRECTORIES(${p}) 
ENDFOREACH(header_file ${folder_header})  


INCLUDE_DIRECTORIES(/home/bishop/iCubDev/ARToolKit/include/AR/ ${OPENCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
LINK_LIBRARIES(/home/bishop/iCubDev/ARToolKit/lib/libAR.a ${OPENCV_LIBRARIES} ${ICUB_LIBRARIES} ${YARP_LIBRARIES})
# Set up our main executable. 
IF (folder_source)   
	ADD_EXECUTABLE(${KEYWORD} ${folder_source} ${folder_header}) 
ELSE (folder_source)   
	MESSAGE(FATAL_ERROR "No source code files found. Please add something") 
ENDIF (folder_source) 

INSTALL_TARGETS(../../bin ${KEYWORD})
