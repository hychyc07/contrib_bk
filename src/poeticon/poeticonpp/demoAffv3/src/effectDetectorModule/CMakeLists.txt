cmake_minimum_required(VERSION 2.6)
SET(KEYWORD "effectDetector")
PROJECT(${KEYWORD})

FIND_PACKAGE(YARP)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

FIND_PACKAGE(OpenCV)

INCLUDE_DIRECTORIES(
		${OpenCV_INCLUDE_DIRS}
		${YARP_INCLUDE_DIRS}) 

SET(folder_source src/yarpTimer.cpp src/effectDetector.cpp src/effectDetectorMain.cpp)
SET(folder_header include/iCub/yarpTimer.h include/iCub/effectDetector.h)
INCLUDE_DIRECTORIES(./include)

ADD_EXECUTABLE(effectDetector ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${KEYWORD} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

