cmake_minimum_required(VERSION 2.6)
 
set(PROJECTNAME tactileLearning)
PROJECT(${PROJECTNAME})

#find packages
find_package(YARP)
find_package(ICUB)
find_package(Boost REQUIRED)

#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

#inherit options from the iCub build (inlcudes rpath settings and disables some warnings in msvc

#load some macros useful for applications (see below)
include(iCubHelpers)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories($ENV{ICUB_ROOT}/iCub/contrib/src/piSquare/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories($ENV{EIGEN_INCLUDE_PATH})

#Search for source code.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/tactileLearning/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
 
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} $ENV{ICUB_ROOT}/iCub/contrib/src/piSquare/build/Debug/piSquared.lib)


INSTALL(TARGETS ${PROJECTNAME}  DESTINATION bin)


