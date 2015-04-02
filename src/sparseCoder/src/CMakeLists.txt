# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME sparseCoder)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(SIFTGPU)

if(SIFTGPU_FOUND)
   find_package(YARP)
   find_package(ICUB)
   set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
   set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
   INCLUDE(iCubOptions)
   include(iCubHelpers)
   
   find_package(OpenCV)
      
   set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
   
   include_directories(${YARP_INCLUDE_DIRS})
   include_directories(${OpenCV_INCLUDE_DIRS})
   include_directories(${boostMIL_INCLUDE_DIRS})
      
   set(folder_source main.cpp SiftGPU_Extractor.cpp DictionaryLearning.cpp)
   set(folder_header SiftGPU_Extractor.h DictionaryLearning.h)
   source_group("Source Files" FILES ${folder_source})
   source_group("Header Files" FILES ${folder_header})
   include_directories(${PROJECT_SOURCE_DIR} ${SIFTGPU_INCLUDE_DIRS})
   add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
   target_link_libraries(${PROJECTNAME} boostMIL ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${SIFTGPU_LIBRARIES})
   install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)   
endif()

