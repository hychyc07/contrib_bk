# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

PROJECT(Darwin)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.
FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(IPOPT)
# find lapack and vlfeat (TODO)


set(ICUB_APPLICATIONS_PREFIX $ENV{ICUB_ROOT} CACHE PATH "Prefix directory for installing applications. Read only, set ICUB_ROOT environment variable to affect this setting." FORCE)

set(CMAKE_INSTALL_PREFIX $ENV{ICUB_DIR} CACHE PATH "Prefix directory for installing executables" FORCE)

include(iCubHelpers)
include(iCubOptions)


ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)
icub_app_all()

