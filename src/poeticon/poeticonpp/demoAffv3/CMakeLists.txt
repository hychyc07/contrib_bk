cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set the build output directory
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

find_package(ICUB)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

# Build behavior module and dependencies
ADD_SUBDIRECTORY(src/behaviorControlModule/lib/attentionSystem/)
ADD_SUBDIRECTORY(src/behaviorControlModule)

# Build affordance module and dependencies
ADD_SUBDIRECTORY(src/affordanceControlModule)

#Build effect detector module and dependencies
ADD_SUBDIRECTORY(src/effectDetectorModule)

add_subdirectory(src/controlGaze)
