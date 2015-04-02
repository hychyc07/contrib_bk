IF(WIN32)
cmake_minimum_required(VERSION 2.8)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

PROJECT(crawling)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

# now load support files 
include(iCubOptions)
include(iCubHelpers)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})

ADD_SUBDIRECTORY(src/crawlGenerator)
ADD_SUBDIRECTORY(src/crawlManager)
ADD_SUBDIRECTORY(src/crawlHeadControl)
ADD_SUBDIRECTORY(src/velImpControl)
ADD_SUBDIRECTORY(src/planner)
#ADD_SUBDIRECTORY(src/reachManager)
#ADD_SUBDIRECTORY(src/velImpControl2)

add_subdirectory(app)

IF(MSVC)
	add_definitions( /wd4250 )
ENDIF(MSVC)

FIND_PACKAGE(ARToolkitPlus QUIET)
IF (ARTOOLKITPLUS_FOUND)
  ADD_SUBDIRECTORY(multiMarkerTracker)
ELSEIF (ARTOOLKITPLUS_FOUND)
  MESSAGE(STATUS "Skipping module MultiMarkerTracker using ARToolkitPlus")
ENDIF (ARTOOLKITPLUS_FOUND)


