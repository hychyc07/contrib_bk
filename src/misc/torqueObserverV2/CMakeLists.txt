SET(PROJECTNAME torqueObserverV2)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source main.cpp
		  torqueObserver.cpp
		  armDynamics.cpp)

SET(folder_header torqueObserver.h
		  armDynamics.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     ${YARP_LIBRARIES})

# INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)





