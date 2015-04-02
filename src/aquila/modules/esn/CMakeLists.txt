# Copyright: 2012 RobotCub Consortium
# Author: Anthony Morse
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME esn)
project(${PROJECTNAME})

set(INCLUDES include/interface.h)
set(SOURCES  src/main.cpp src/interface.cpp)

source_group("Header Files" FILES ${INCLUDES})
source_group("Source Files" FILES ${SOURCES})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${QT_INCLUDES}
                    ${AQUILA_LIB_DIR}/aquila-nnet/include
                    ${AQUILA_LIB_DIR}/aquila-utility/include
                    ${CUDA_INCLUDE_DIRS}
		    ${YARP_INCLUDE_DIRS})

include(${QT_USE_FILE})
qt4_wrap_cpp(MOC_SOURCES ${INCLUDES})

add_executable(${PROJECTNAME} ${MOC_SOURCES} ${INCLUDES} ${SOURCES})

target_link_libraries(${PROJECTNAME} aquila-nnet aquila-utility ${QT_LIBRARIES} ${CUDA_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)		
