#cmake_minimum_required(VERSION 2.8)
#project(hmin++)
set(HMINLIBRARY hmin++ CACHE INTERNAL "name of the hmin++ library")
set(HDRDIR ./include/)
set(SRCDIR ./src/)

set(HMIN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ CACHE INTERNAL "path to the hmin include directories")

#find_package(Gurls)
#include_directories(${GURLS_INCLUDE_DIRS})

set(hmin_headers
  ${HDRDIR}layer.h
  ${HDRDIR}filter.h
  ${HDRDIR}gaborfilter.h
  ${HDRDIR}maxfilter.h
  ${HDRDIR}hmaxModel.h
  ${HDRDIR}ndpfilter.h
  ${HDRDIR}gmaxfilter.h)

set(hmin_sources
  ${SRCDIR}hmaxModel.cpp
  ${SRCDIR}filter.cpp
  ${SRCDIR}layer.cpp
  ${SRCDIR}gaborfilter.cpp
  ${SRCDIR}maxfilter.cpp
  ${SRCDIR}ndpfilter.cpp
  ${SRCDIR}gmaxfilter.cpp
  hminModule.cpp)

include_directories(${HDRDIR})

add_executable(hmin_test ${hmin_headers} ${hmin_sources})
#link_directories(${GURLS_LIB_DIR})
#target_link_libraries(test ${GULRS_LIBS} ${OpenCV_LIBS})
target_link_libraries(hmin_test ${OpenCV_LIBS} ${YARP_LIBRARIES})

add_library(${HMINLIBRARY} ${hmin_headers} ${hmin_sources})
target_link_libraries(${HMINLIBRARY} ${OpenCV_LIBS} ${YARP_LIBRARIES})
