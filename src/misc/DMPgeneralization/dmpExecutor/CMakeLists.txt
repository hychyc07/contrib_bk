# Copyright: (C) 2013 Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME dmpExecutor)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})  

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${DynamicMovementPrimitives_INCLUDE_DIRS}
                    ${thriftGeneratedDMP_INCLUDE_DIRS}
					${ctrlLib_INCLUDE_DIRS})


# Search for source code.
set(folder_source src/main.cpp src/module.cpp src/utils.cpp src/KinestheticTeaching.cpp)
set(folder_header include/iCub/module.h include/iCub/utils.h include/iCub/KinestheticTeaching.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# add executables and link libraries.
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ctrlLib ${YARP_LIBRARIES} icubmod thriftGeneratedDMP DynamicMovementPrimitives )
install(TARGETS ${PROJECTNAME} DESTINATION bin)

