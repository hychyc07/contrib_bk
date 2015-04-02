cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME iKartBatteryDisplay)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(${GtkMM_LIBRARIES})
LINK_DIRECTORIES(${GtkMM_LIBRARY_DIR})
INCLUDE_DIRECTORIES(${GtkMM_INCLUDE_DIRS})


INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 
                      ${GtkMM_LINK_FLAGS}
                      ${GtkMM_LIBRARIES}
                      ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
