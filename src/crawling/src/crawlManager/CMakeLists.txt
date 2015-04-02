IF(WIN32)
cmake_minimum_required(VERSION 2.8)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(KEYWORD "crawlManager")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})


FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

IF(folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

SOURCE_GROUP("Source Files" FILES ${folder_source})
TARGET_LINK_LIBRARIES(${KEYWORD} ${YARP_LIBRARIES})
INSTALL(TARGETS ${KEYWORD} DESTINATION ${ICUB_DIR}/bin)

