#Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project (morphoGen)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf) 

#find_package(IPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(GSL)
find_package(IPOPT)

INCLUDE(iCubOptions)
include(iCubHelpers)
icub_install_with_rpath()

SET(MORPHOGENLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/attentionLib/include)

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single folder
#option(ENABLE_CUDA  "Do you want to compile with Cuda?" OFF)

#if (ENABLE_CUDA)  
#    find_package(CUDA QUIET REQUIRED)
#    add_definitions (-DWITH_CUDA)
#    add_subdirectory(cudaVision)
#endif (ENABLE_CUDA)

#ADD_SUBDIRECTORY(morphoGenLib)
add_subdirectory(app)
add_subdirectory(src)

