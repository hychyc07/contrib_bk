IF(WIN32)
cmake_minimum_required(VERSION 2.6)
#disable annoying warnings about use of deprecated c functions
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
#removed MSVCRT from /NODEFAULTLIB to allow YARP to link
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libc;libcmt.lib;libcmtd.lib") 
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)


SET(MODULENAME smoothPursuit)
PROJECT(${MODULENAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

FILE(GLOB folder_source src/smoothPursuit.cpp src/smoothPursuitMain.cpp)
FILE(GLOB folder_header include/iCub/smoothPursuit.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${MODULENAME} ${folder_source} ${folder_header})
 
TARGET_LINK_LIBRARIES(${MODULENAME} ${YARP_LIBRARIES})

INSTALL(TARGETS ${MODULENAME} DESTINATION bin)