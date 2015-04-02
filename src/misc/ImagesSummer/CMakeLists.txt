cmake_minimum_required(VERSION 2.6)
 ## just for avoid annoying warning when using CMake > 2.6
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(MODULENAME "ImagesSummer")

find_package(YARP)
find_package(ICUB)
 
#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
 
#find packages
find_package(OpenCV)

INCLUDE_DIRECTORIES(	${PROJECT_SOURCE_DIR}/include
						${YARP_INCLUDE_DIRS}
						${ICUB_INCLUDE_DIRS}
						${OpenCV_INCLUDE_DIRS}
						)
 
# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
ADD_EXECUTABLE(${MODULENAME} ${folder_source} ${folder_header})


target_link_libraries(${MODULENAME}  	${YARP_LIBRARIES}
										${OpenCV_LIBRARIES}
										)


SET_TARGET_PROPERTIES(${KEYWORD} PROPERTIES COMPILE_FLAGS /wd4996)