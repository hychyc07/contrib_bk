# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME dmpManager)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})  

find_package(OpenCV)
#find_package(GSL)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS} 
 #                   ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${DynamicMovementPrimitives_INCLUDE_DIRS}
                    ${thriftGeneratedDMP_INCLUDE_DIRS})

message ("gsl incl dir ${GSL_INCLUDE_DIRS}")
# Search for source code.
set(folder_source src/main.cpp src/module.cpp src/utils.cpp)
set(folder_header include/iCub/module.h include/iCub/utils.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# add executables and link libraries.
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} thriftGeneratedDMP DynamicMovementPrimitives)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

