# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME dmpLearner)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
   
include(iCubHelpers)

include_directories(${PROJECT_SOURCE_DIR}/include
		            ${dmpGPR_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${thriftGeneratedDMP_INCLUDE_DIRS}
                   # ${ICUB_INCLUDE_DIRS}
		)

message("dmpGPR incl dirs: ${dmpGPR_INCLUDE_DIRS}")
# Search for source code.
set(folder_source src/main.cpp src/module.cpp src/utils.cpp)
set(folder_header include/iCub/module.h include/iCub/utils.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# add executables and link libraries.
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME}  ${YARP_LIBRARIES} 
		#${ICUB_LIBRARIES} 
		dmpGPR thriftGeneratedDMP)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

