# Copyright: (C) 2010 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

# set up our program
add_executable(movTut main.cpp Container.cpp EndeffectorPos.cpp)

# link with YARP libraries
target_link_libraries(movTut ${YARP_LIBRARIES})
target_link_libraries(movTut ${ICUB_LIBRARIES})
