CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME activeObjRecControl)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(Boost)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

# Create everything needed to build our executable.
ADD_EXECUTABLE(activeObjRecControl ObjRecControlModule.cpp)

TARGET_LINK_LIBRARIES(activeObjRecControl ${YARP_LIBRARIES} ${Boost_LIBRARIES})
