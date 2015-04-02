cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME 3DObjectReconstruction)

project(${PROJECTNAME})

find_package(PCL 1.6 REQUIRED)

find_package(YARP)
find_package(ICUB)

set (thrift3D_DIR ../thrift3D/build CACHE PATH "The directory containing a CMake configuration file for the thrift3D generated library")
find_package(thrift3D REQUIRED)

set (minimumBoundingBox_DIR ../minimumBoundingBox/build CACHE PATH "The directory containing a CMake configuration file for the minimumBoundingBox library")
find_package(minimumBoundingBox REQUIRED)

set (thriftGBseg_DIR ../segmentation/graphBased/build CACHE PATH "The directory containing a CMake configuration file for the thriftGBseg generated library")
find_package(thriftGBseg REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(OpenCV)

include(iCubOptions)
include(iCubHelpers)

set (stereoVision_DIR ../stereoVision/src/lib/build CACHE PATH "The directory containing a CMake configuration file for the stereoVision library")
find_package(stereoVision)

include_directories(${PCL_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${stereoVision_INCLUDE_DIRS} ${thriftGBseg_INCLUDE_DIRS} ${thrift3D_INCLUDE_DIRS} ${minimumBoundingBox_INCLUDE_DIRS})

#file(GLOB source src/*.cpp)
set(source src/main.cpp src/communicator.cpp src/objectReconstr.cpp src/reconstructionRoutine.cpp)
#file(GLOB header include/*.h)
set(header include/communicator.h include/objectReconstr.h include/reconstructionRoutine.h)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} thriftGBseg stereoVision thrift3D minimumBoundingBox icubmod ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
