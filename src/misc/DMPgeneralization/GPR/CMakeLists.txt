# Copyright: (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME GPRLearner)

PROJECT(${PROJECTNAME})

find_package(IPOPT)
if (IPOPT_FOUND)
message("IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${IPOPT_INCLUDE_DIRS})
set(GPR_USE_IPOPT TRUE) # need it to configure GPRoptimization.h
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/GPRoptimization.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/iCub/learningMachine/GPRoptimization.h)
SET(folder_source src/GPR.cpp src/Mean.cpp src/LinearMean.cpp src/Covariance.cpp src/CovSEard.cpp src/GPRoptimization.cpp)
SET(folder_header include/iCub/learningMachine/GPR.h include/iCub/learningMachine/Mean.h include/iCub/learningMachine/LinearMean.h include/iCub/learningMachine/Covariance.h include/iCub/learningMachine/CovSEard.h include/iCub/learningMachine/GPRoptimization.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${ICUB_LIBRARIES} ${YARP_LIBRARIES})
IF(GPR_USE_IPOPT)
   TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IPOPT_LIBRARIES})
   icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include  #should add ${YARP_INCLUDE_DIRS} as EXTERNAL_INCLUDE_DIRS ...?
                                   EXTERNAL_INCLUDE_DIRS ${IPOPT_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS}
                                   DEPENDS IPOPT ICUB
                                   DESTINATION include/iCub/learningMachine # THRIFT_HEADERS already have the include/iCub/etc path
                                   FILES ${folder_header} VERBOSE) 
ENDIF(GPR_USE_IPOPT)
#set(GPRLearner_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "GPRLearner include directory")  

