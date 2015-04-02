# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# find matlab libraries
find_path(MATLAB_INCLUDE_DIRS engine.h NO_DEFAULT_PATH)
if(WIN32)
   find_path(MATLAB_LIBRARIES_DIRS libeng.lib NO_DEFAULT_PATH)
else(WIN32)
   find_path(MATLAB_LIBRARIES_DIRS eng NO_DEFAULT_PATH)
endif(WIN32)

if(NOT MATLAB_LIBRARIES_DIRS)
   message(FATAL_ERROR "Unable to locate MATLAB directories")
endif()

if(WIN32)
   set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.lib
                                                ${MATLAB_LIBRARIES_DIRS}/libmx.lib
                                                ${MATLAB_LIBRARIES_DIRS}/libmex.lib
                                                ${MATLAB_LIBRARIES_DIRS}/libmat.lib)
else(WIN32)
   set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.so
                                                ${MATLAB_LIBRARIES_DIRS}/libmx.so
                                                ${MATLAB_LIBRARIES_DIRS}/libmex.so
                                                ${MATLAB_LIBRARIES_DIRS}/libmat.so)
endif(WIN32)

add_subdirectory(iKinArmView)
add_subdirectory(iKinGazeView)
add_subdirectory(signalScope)

