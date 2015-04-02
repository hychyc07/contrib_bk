cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME poeticonReacher)

project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

set(folder_source main.cpp)

source_group("Source Files" FILES ${folder_source})

include_directories(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
					
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

add_executable(${PROJECTNAME} ${folder_source})

target_link_libraries(${PROJECTNAME} icubmod ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
								 
