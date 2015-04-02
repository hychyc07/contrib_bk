# Copyright: (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#reduce warning level with cmake 2.8
cmake_minimum_required(VERSION 2.8)
#cmake policies
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(PROJECTNAME DMPgeneralization)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

# add yarp definitions
add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT

# pick up yarp's cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

find_package(ICUB REQUIRED)

# pick up iCub's cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

## Add generated libraries first
add_subdirectory(thrift)
add_subdirectory(generated)

## then libraries
add_subdirectory(GPR)
add_subdirectory(DynamicMovementPrimitives)
add_subdirectory(dmpGPR)

## then modules
add_subdirectory(dmpManager)
add_subdirectory(dmpLearnerModule)
add_subdirectory(dmpExecutor)
# and finally applications
add_subdirectory(app)

#add tests
add_subdirectory(tests)
