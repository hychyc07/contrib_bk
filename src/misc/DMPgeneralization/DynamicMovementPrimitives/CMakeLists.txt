# Copyright: (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME DynamicMovementPrimitives)

PROJECT(${PROJECTNAME})
SET(folder_source src/DMPPointToPoint.cpp src/DMPPeriodic.cpp src/DMP.cpp)
SET(folder_header include/iCub/DMP/DMP.h include/iCub/DMP/DMPPointToPoint.h include/iCub/DMP/DMPPeriodic.h include/iCub/DMP/Trajectory.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

message ("gsl include dir: ${GSL_INCLUDE_DIR}, gsl include dirs: ${GSL_INCLUDE_DIRS}, yarp include dirs: ${YARP_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ctrlLib_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib ${ICUB_LIBRARIES})
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/DMP 
                                   FILES ${folder_header} VERBOSE) 
#set(DynamicMovementPrimitives_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "DynamicMovementPrimitives
#     include directory")  

