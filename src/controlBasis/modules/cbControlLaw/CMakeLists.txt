#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME cbControlLaw)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf ${ICUB_DIR}/conf)

SET(ICUB_CONTRIB $ENV{ICUB_ROOT}/contrib)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${ICUB_CONTRIB}/include)
INCLUDE_DIRECTORIES(${ICUB_CONTRIB}/build/include)

LINK_DIRECTORIES(${ICUB_CONTRIB}/lib)
LINK_DIRECTORIES(${ICUB_CONTRIB}/build/lib)


ADD_EXECUTABLE(cbControlLaw controlBasisControlLaw.cpp)

TARGET_LINK_LIBRARIES(${PROJECTNAME} controlBasis
                                     rl
                                     ${ICUB_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

