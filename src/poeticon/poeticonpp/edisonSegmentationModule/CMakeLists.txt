# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME "edisonSegmenter")
PROJECT(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

FIND_PACKAGE(YARP)

FIND_PACKAGE(ICUB)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
INCLUDE(iCubOptions)
include(iCubHelpers)

FIND_PACKAGE(OpenCV)

INCLUDE_DIRECTORIES(
		${OpenCV_INCLUDE_DIRS}
		${YARP_INCLUDE_DIRS}) 

FILE(GLOB module_source *.cpp)
FILE(GLOB module_header *.h)
SET(edison_source_prompt edison_source/prompt/parser.cpp edison_source/prompt/globalFnc.cpp edison_source/prompt/edison.cpp edison_source/prompt/libppm.cpp)
SET(edison_header_prompt edison_source/prompt/parser.h edison_source/prompt/defs.h edison_source/prompt/error.h edison_source/prompt/flags.h edison_source/prompt/edison.h edison_source/prompt/libppm.h edison_source/prompt/imfilt.h)
SET(edison_source_segm edison_source/segm/ms.cpp edison_source/segm/msSysPrompt.cpp edison_source/segm/msImageProcessor.cpp edison_source/segm/RAList.cpp edison_source/segm/rlist.cpp)
SET(edison_header_segm edison_source/segm/tdef.h edison_source/segm/RAList.h edison_source/segm/ms.h edison_source/segm/msImageProcessor.h edison_source/segm/msSys.h edison_source/segm/rlist.h)
SET(edison_source_edge edison_source/edge/BgEdge.cpp edison_source/edge/BgEdgeDetect.cpp edison_source/edge/BgEdgeList.cpp edison_source/edge/BgGlobalFc.cpp edison_source/edge/BgImage.cpp)
SET(edison_header_edge edison_source/edge/BgDefaults.h edison_source/edge/BgEdge.h edison_source/edge/BgEdgeDetect.h edison_source/edge/BgImage.h)
SET(folder_source ${module_source} ${edison_source_prompt} ${edison_source_edge} ${edison_source_segm})
SET(folder_header ${module_header} ${edison_header_prompt} ${edison_header_edge} ${edison_header_segm})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

# Automatically add include directories if needed.
FOREACH(header_file ${folder_header})
  GET_FILENAME_COMPONENT(p ${header_file} PATH)
  INCLUDE_DIRECTORIES(${p})
ENDFOREACH(header_file ${folder_header})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

add_subdirectory(app)

