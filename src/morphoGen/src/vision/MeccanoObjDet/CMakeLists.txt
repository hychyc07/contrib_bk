# Copyright: (C) 2013 Robotics and Brain Cognitive Science - RBCS
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include(${ROBVIS_CMAKE_DIRECTORY}/externals/lib_opencv/head.cmake)
#include(${CMAKE_CURRENT_SOURCE_DIR}/../yarp/CMakeLists.txt)

#robvis_create_executable_project(ShapeDet)

#robvis_target_link_libraries(cvutShapeDet ${OPENCV_ALL_TARGETS})
#robvis_target_link_libraries(cvutShapeDet ${YARP_ALL_TARGETS})

SET(KEYWORD "MeccanoObjDet")
PROJECT(${KEYWORD})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(YARP REQUIRED)
#LINK_LIBRARIES(${YARP_LIBRARIES})
#INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})


FIND_PACKAGE(ICUB REQUIRED)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubHelpers)

FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}
	${YARP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/*.h include/*.hpp )

MESSAGE("folder_header ${folder_header}")

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}        
     ${YARP_LIBRARIES}
     ${OPENCV_LIBRARIES}
     ${OpenCV_LIBS}
    )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
