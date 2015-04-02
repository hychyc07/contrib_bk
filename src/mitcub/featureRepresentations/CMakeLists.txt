#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(FEAT_REP_LIB feat_rep CACHE INTERNAL "name of the feature representation collection library")
#PROJECT(${FEAT_REP_LIB})

set(SRCDIR ./src/)
set(HDRDIR ./include/)

set(FEAT_REP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ CACHE INTERNAL "path to the hmin include directories")

#FIND_PACKAGE(YARP)
#FIND_PACKAGE(ICUB)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
#INCLUDE(iCubHelpers)

#FIND_PACKAGE(OpenCV)

set(folder_header
  ${HDRDIR}Extractor.h
  ${HDRDIR}ExtractorFactory.h
  ${HDRDIR}DummyExtractor.h
  ${HDRDIR}HmaxExtractor.h
  ${HDRDIR}HighDimensionalityFeatureExtractor.h
  ${HDRDIR}BagOfWordsExtractor.h
  ${HDRDIR}ExtractorUtils.h)

set(folder_source
  ${SRCDIR}Extractor.cpp
  ${SRCDIR}HmaxExtractor.cpp
  ${SRCDIR}HighDimensionalityFeatureExtractor.cpp
  ${SRCDIR}BagOfWordsExtractor.cpp
  ${SRCDIR}ExtractorUtils.cpp)


include_directories(${HDRDIR})
#INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

include_directories(${HMIN_INCLUDE_DIRS})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

ADD_LIBRARY(${FEAT_REP_LIB} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${FEAT_REP_LIB} ${YARP_LIBRARIES} ctrlLib ${ICUB_LIBRARIES} ${HMINLIBRARY})
