CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME featureExtractor)
#set(SRCDIR ./src/)
#set(HDRDIR ./include/)

#PROJECT(${PROJECTNAME})

#FIND_PACKAGE(YARP)
#FIND_PACKAGE(ICUB)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
#INCLUDE(iCubHelpers)

#FIND_PACKAGE(OpenCV)

set(folder_header ExtractorModule.h)

set(folder_source main.cpp ExtractorModule.cpp)


#INCLUDE_DIRECTORIES(${HDRDIR})
#include_directories(${HMIN_INCLUDE_DIRS})
include_directories(${FEAT_REP_INCLUDE_DIRS})

#INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib ${ICUB_LIBRARIES} ${FEAT_REP_LIB})

install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

add_subdirectory(app)
