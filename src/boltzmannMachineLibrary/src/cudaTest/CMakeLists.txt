# Copyright: (C) 2010 RobotCub Consortium
# Authors: Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "cudaTest")
PROJECT(${KEYWORD})


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)


# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c src/*.cu)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#Include the FindCUDA script
#INCLUDE(FindCUDA)
MESSAGE("CUDA_SDK_ROOT_DIR ${CUDA_SDK_ROOT_DIR}")
INCLUDE_DIRECTORIES(${CUDA_SDK_ROOT_DIR}/C/common/inc)
INCLUDE_DIRECTORIES(${CUDA_SDK_ROOT_DIR}/shared/inc)

#Linking with libcutil (expected to be in ${CUDA_SDK_ROOT_DIR}/C/lib)
#LINK_LIBRARIES("-L/${CUDA_SDK_ROOT_DIR}/C/lib -lcutil")


#Linking with libcutil (expected to be in ${CUDA_SDK_ROOT_DIR}/C/lib)
find_library(CUDA_CUTIL_LIBRARY util "${CUDA_SDK_ROOT_DIR}/C/lib")
MESSAGE("CUDA_CUTIL_LIBRARY ${CUDA_CUTIL_LIBRARY}")
find_library(CUDA_SHRUTIL_LIBRARY shrutil "${CUDA_SDK_ROOT_DIR}/shared/lib")
#LINK_LIBRARIES("-L/${CUDA_SDK_ROOT_DIR}/C/lib -lcutil")



#The rule to build the executable program matrixMul from matrixmul.cu
# and matrixMul_gold.cpp
IF (folder_source)
  CUDA_ADD_EXECUTABLE(${KEYWORD} 
    ${folder_source} 
    ${folder_header} 
    )
  target_link_libraries(${KEYWORD} ${CUDA_CUTIL_LIBRARY}
    ${CUDA_SDK_ROOT_DIR}/shared/lib/libshrutil.a
    ${CUDA_SDK_ROOT_DIR}/C/lib/libutil.a
    ${YARP_LIBRARIES}
    )
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
