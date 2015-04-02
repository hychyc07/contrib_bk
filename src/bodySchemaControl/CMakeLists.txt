# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea, Dalia De Santis
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project (bodySchemaControl)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf) 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

find_package(GSL)
find_package(OpenCV REQUIRED)
find_package(GtkPlus)
find_package(IPOPT)

include(iCubOptions)
include(iCubHelpers)

SET(BODYSCHEMALIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/bodySchemaLib/include)

#default prefix for executables
set(CMAKE_INSTALL_PREFIX $ENV{ICUB_DIR} CACHE PATH "Prefix directory for installing executables" FORCE)

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX $ENV{ICUB_ROOT} CACHE PATH "Prefix directory for installing applications. Read only, set ICUB_ROOT environment variable to affect this setting." FORCE)

### Go through single folder
option(ENABLE_CUDA  "Do you want to compile with Cuda?" OFF)

if (ENABLE_CUDA)  
    find_package(CUDA QUIET REQUIRED)
    add_definitions (-DWITH_CUDA)
    add_subdirectory(cudaVision)
endif (ENABLE_CUDA)

add_subdirectory(bodySchemaLib)
add_subdirectory(app)
add_subdirectory(src)

