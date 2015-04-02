# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak
# CopyPolicy: Released under the terms of the FreeBSD license

set(LIBNAME aquila-utility)
project(${LIBNAME})

set(INCLUDES include/aquila-utility.h)
set(SOURCES src/aquila-utility.cpp)
set(CUDA_INCLUDES include/kernels.h)
set(CUDA_SOURCES src/kernels.cu)

source_group("Header Files" FILES ${INCLUDES} ${CUDA_INCLUDES})
source_group("Source Files" FILES ${SOURCES} ${CUDA_SOURCES})

include_directories(${PROJECT_SOURCE_DIR}/include ${QT_INCLUDES} ${CUDA_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include(${QT_USE_FILE})
qt4_wrap_cpp(MOC_SOURCES ${INCLUDES})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-arch=sm_20 -use_fast_math")
cuda_add_library(${LIBNAME} ${CUDA_SOURCES} ${MOC_SOURCES} ${INCLUDES} ${SOURCES})

target_link_libraries(${LIBNAME} ${QT_LIBRARIES} ${CUDA_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${LIBNAME} DESTINATION lib)



