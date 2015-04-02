cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME GuiBalancer)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

FILE(GLOB folder_source main.cpp)
FILE(GLOB folder_header *.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubOptions)
include(iCubHelpers)

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} 
					${YARP_INCLUDE_DIRS} 
					${ctrlLib_INCLUDE_DIRS})

# Create everything needed to build our executable.
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib icubmod)
