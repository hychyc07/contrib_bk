# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME iCubWriter)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
include(iCubHelpers)

set(TinyXML_INCLUDE_DIRS "$ENV{YARP_ROOT}/extern/tinyxml/tinyxml" CACHE PATH "TinyXML include directory" FORCE)
set(TinyXML_CODE ${TinyXML_INCLUDE_DIRS}/tinystr.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxml.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxmlerror.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxmlparser.cpp)

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

add_executable(${PROJECTNAME} src/main.cpp ${TinyXML_CODE})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} actionPrimitives)
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

########## application
icub_app(${PROJECTNAME})
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
icub_app_install(${PROJECTNAME} FILES ${conf} DESTINATION conf)

