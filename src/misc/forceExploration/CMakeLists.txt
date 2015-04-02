SET(PROJECTNAME forceExploration)
PROJECT(${PROJECTNAME})

cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# add include directories
INCLUDE_DIRECTORIES(. ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ./include)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

ADD_EXECUTABLE(${PROJECTNAME} src/main.cpp ./include/armExploration.h ./src/armExploration.cpp ./include/wrenchComputation.h ./src/wrenchComputation.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} iDyn icubmod ${ICUB_LIBRARIES})


