CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME ifar)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)


#FIND_PACKAGE(LTILIB REQUIRED)

SET(CMAKE_MODULE_PATH ${YARP_DIR}/conf ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_DIR}/conf ${CMAKE_MODULE_PATH})
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")	


include(iCubOptions)
include(iCubHelpers)

FIND_PACKAGE(OpenCV REQUIRED)				


# In addition to specifying the libraries to link, the directories containing the
# libraries should be added in case the libraries are in a non-standard location

LINK_DIRECTORIES(${OpenCV_LINK_DIRECTORIES})
LINK_LIBRARIES(${OpenCV_LIBS})
LINK_LIBRARIES(${OpenCV_BIN})

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR} ${OpenCV_INCLUDE_DIR_CV})

MESSAGE("OpenCV_DIRS : ${OpenCV_INCLUDE_DIR_CV}")					

MESSAGE("OpenCV_LIBRARIES : ${OpenCV_LIBS}")					

#Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c *.cpp)
FILE(GLOB folder_header include/iCub/*.h )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

MESSAGE("folder_source ${folder_header}")

#SET(OPENCV_LIBRARIES "/home/rea/work/OpenCV-2.0.0/lib

# Create everything needed to build our executable.
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} )

TARGET_LINK_LIBRARIES(ifar ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${OpenCV_LIBS})
#TARGET_LINK_LIBRARIES(ifar msvcrt.lib)
