# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak <www.martinpeniak.com>
# CopyPolicy: Released under the terms of the FreeBSD license.

cmake_minimum_required(VERSION 2.8)
project(Aquila)

# find packages  
find_package(Qt4 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(YARP REQUIRED)
find_package(OpenMP REQUIRED)

# set flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# disable MSVC warnings
if(MSVC)
   add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif(MSVC)

set(AQUILA_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries)

# put binaries under aquila/bin
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
message(STATUS "Executables will be placed in ${EXECUTABLE_OUTPUT_PATH}")

# put libraries under aquila/lib
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")

# add aquila
add_subdirectory(aquila) 

# add aquila libraries
add_subdirectory(libraries)

# add aquila modules
add_subdirectory(modules)



