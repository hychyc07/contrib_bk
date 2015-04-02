# Copyright: (C) 2010 RobotCub Consortium
# Authors: Christian Dondrup
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project (SpeechSegmenter)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

INCLUDE(FindPkgConfig)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
pkg_check_modules(FFTW3 REQUIRED fftw3)
pkg_check_modules(SNDFILE REQUIRED sndfile)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")

#find_package(IPP REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(GtkPlus REQUIRED)
#find_package(IPOPT)

include(iCubOptions)
include(iCubHelpers)

add_subdirectory(src)
add_subdirectory(doc)

