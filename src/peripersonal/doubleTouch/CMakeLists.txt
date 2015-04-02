cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME iCubDoubleTouch)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)
include(iCubOptions)

find_package(IPOPT REQUIRED)

set(folder_source ./src/main.cpp ./src/iCubDblTchSlv.cpp ./src/iCubDblTchThrd.cpp ./src/iKinFwdMod.cpp)
set(folder_header ./include/iCubDblTchSlv.h ./include/iCubDblTchThrd.h ./include/iKinFwdMod.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iKin_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} iKin ${ICUB_LIBRARIES} ${IPOPT_LIBRARIES})
