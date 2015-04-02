cmake_minimum_required(VERSION 2.6)


set(PROJECTNAME piSquareDemo)

PROJECT(${PROJECTNAME})




find_package(YARP)
find_package(ICUB)

# Search for source code.

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})




include_directories(${PROJECT_SOURCE_DIR})

include_directories(${YARP_INCLUDE_DIRS})


include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/../include)
#include_directories(${piSquare_INCLUDE_DIRS})
include_directories("C:/Program Files/eigen/eigen-eigen-6e7488e20373")
include_directories("C:/Program Files/boost/boost_1_44")




ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} 
			"C:/Program Files/robotology/icub-1.1.9/iCub/contrib/src/piSquare/build/Debug/piSquared.lib")


INSTALL(TARGETS ${PROJECTNAME}  DESTINATION bin)

