cmake_minimum_required(VERSION 2.6)


set(PROJECTNAME piSquareDemo2)

PROJECT(${PROJECTNAME})




find_package(YARP)
find_package(ICUB)
find_package(Boost REQUIRED)

# Search for source code.

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})




include_directories(${PROJECT_SOURCE_DIR})

include_directories(${YARP_INCLUDE_DIRS})


include_directories(${ICUB_INCLUDE_DIRS})
include_directories($ENV{ICUB_ROOT}/iCub/contrib/src/piSquare/include)
#include_directories(${piSquare_INCLUDE_DIRS})
include_directories($ENV{EIGEN_INCLUDE_PATH})
include_directories(${Boost_INCLUDE_DIRS})




ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} $ENV{ICUB_ROOT}/iCub/contrib/src/piSquare/build/Debug/piSquared.lib)


INSTALL(TARGETS ${PROJECTNAME}  DESTINATION bin)

