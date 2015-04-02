IF(WIN32)
cmake_minimum_required(VERSION 2.8)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.6)
ENDIF(WIN32)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET(KEYWORD "crawlGeneratorModule")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(${iKin_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF(folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

SOURCE_GROUP("Source Files" FILES ${folder_source})

TARGET_LINK_LIBRARIES(${KEYWORD} ctrlLib iKin)
INSTALL(TARGETS ${KEYWORD} DESTINATION ${ICUB_DIR}/bin)


