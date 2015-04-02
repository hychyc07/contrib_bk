# Copyright: 2012 RobotCub Consortium
# Author: YourName
# CopyPolicy: Released under the terms of the FreeBSD license.

set(PROJECTNAME blank)
project(${PROJECTNAME})

set(MOC_INCLUDES include/interface.h include/blank.h)
set(INCLUDES ${MOC_INCLUDES} include/kernels.h)
set(SOURCES src/main.cpp src/interface.cpp src/blank.cpp src/kernels.cu)

source_group("Header Files" FILES ${INCLUDES})
source_group("Source Files" FILES ${SOURCES})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${QT_INCLUDES}
                    ${AQUILA_LIB_DIR}/aquila-utility/include
                    ${CUDA_INCLUDE_DIRS}
		    ${YARP_INCLUDE_DIRS})

include(${QT_USE_FILE})
qt4_wrap_cpp(MOC_SOURCES ${MOC_INCLUDES})

# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-arch=sm_20 -use_fast_math")
cuda_add_executable(${PROJECTNAME} ${MOC_SOURCES} ${INCLUDES} ${SOURCES})

target_link_libraries(${PROJECTNAME} aquila-utility ${QT_LIBRARIES} ${CUDA_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)	
