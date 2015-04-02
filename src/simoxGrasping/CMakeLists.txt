#
# Author: Nikolaus Vahrenkamp
#
cmake_minimum_required(VERSION 2.6)

PROJECT(SimoxGrasping)

GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)

SET(Simox_DIR $ENV{Simox_DIR} CACHE STRING "Choose the path to Simox (install or build).")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${Simox_DIR}/share/Simox/cmake)

# now search simox (The file ${Simox_DIR}/share/Simox/cmake/FindSimox.cmake is used)
FIND_PACKAGE(Simox REQUIRED)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)


SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH} ${YARP_MODULE_PATH})
Message ("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

FIND_PACKAGE(OpenCV)

#Create variable for install path
set(INSTALL_PATH_BIN "${ICUB_DIR}/bin" CACHE PATH 
    "Installation directory for executables"
    )
    
set(INSTALL_PATH_LIB "${ICUB_DIR}/lib" CACHE PATH 
    "Installation directory for libraries"
    )
    
#include(iCubHelpers)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)
