
SET(PROJECTNAME iKartSpeedTest)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}
            ${SDL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
            ${iCubDev_INCLUDE_DIRS})

SET(folder_source main.cpp)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${SDL_LIBRARY} icubmod ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)