# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak
# CopyPolicy: Released under the terms of the FreeBSD license

set(LIBNAME aquila-nnet)
project(${LIBNAME})

set(INCLUDES include/aquila-nnet-mtrnn.h include/aquila-nnet-esn.h include/aquila-nnet-som.h)
set(SOURCES src/aquila-nnet-mtrnn.cpp src/aquila-nnet-esn.cpp src/aquila-nnet-som.cpp)
set(CUDA_INCLUDES include/mtrnnKernels.h include/esnKernels.h include/somKernels.h)
set(CUDA_SOURCES src/mtrnnKernels.cu src/esnKernels.cu src/somKernels.cu)

source_group("Header Files" FILES ${INCLUDES} ${CUDA_INCLUDES})
source_group("Source Files" FILES ${SOURCES} ${CUDA_SOURCES})

include_directories(${PROJECT_SOURCE_DIR}/include ${AQUILA_LIB_DIR}/aquila-utility/include ${QT_INCLUDES} ${CUDA_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include(${QT_USE_FILE})
qt4_wrap_cpp(MOC_SOURCES ${INCLUDES})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-arch=sm_20 -use_fast_math")
cuda_add_library(${LIBNAME} ${CUDA_INCLUDES} ${CUDA_SOURCES} ${MOC_SOURCES} ${INCLUDES} ${SOURCES})

target_link_libraries(${LIBNAME} aquila-utility ${QT_LIBRARIES} ${CUDA_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${LIBNAME} DESTINATION lib)



