# Copyright: (C) 2013 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu  elena.ceseracciu@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.7)

set(PROJECTNAME GurlsWrappers)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(Gurls++ REQUIRED)

# add yarp definitions
add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT
add_definitions(${Gurls++_DEFINITIONS})
 
# pick up yarp's cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

add_subdirectory(thrift)

add_subdirectory(GurlsClassificationBatch)

