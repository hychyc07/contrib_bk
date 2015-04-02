# Copyright: (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME dmpGPR)

PROJECT(${PROJECTNAME})
SET(folder_source src/DmpGPR.cpp)
SET(folder_header include/iCub/DMP/DmpGPR.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${DynamicMovementPrimitives_INCLUDE_DIRS} ${GPRLearner_INCLUDE_DIRS})
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} DynamicMovementPrimitives GPRLearner)
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include  #should add ${YARP_INCLUDE_DIRS} as EXTERNAL_INCLUDE_DIRS ...?
                                   EXTERNAL_INCLUDE_DIRS ${DynamicMovementPrimitives_INCLUDE_DIRS} ${GPRLearner_INCLUDE_DIRS}
				   DEPENDS DynamicMovementPrimitives GPRLearner
                                   DESTINATION include/iCub/DMP 
                                   FILES ${folder_header} VERBOSE) 
#set(${PROJECTNAME}_INCLUDE_DIRS ${DynamicMovementPrimitives_INCLUDE_DIRS} ${GPRLearner_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "dmpGPR
#     include directory")  

