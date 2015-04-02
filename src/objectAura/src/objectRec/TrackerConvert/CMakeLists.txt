# Copyright: (C) 2012 RobotCub Consortium
# Authors: Katrin Solveig Lohan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

# set up our program
add_executable(conTracker src/convertTrackerOutput.cpp)

# link with YARP libraries
target_link_libraries(conTracker ${YARP_LIBRARIES})
target_link_libraries(conTracker ${ICUB_LIBRARIES})
