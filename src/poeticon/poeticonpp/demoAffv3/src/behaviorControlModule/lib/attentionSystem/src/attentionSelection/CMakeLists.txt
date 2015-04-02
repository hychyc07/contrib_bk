CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME attentionSelection)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubHelpers)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(EGOSPHERELIB REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${EGOSPHERELIB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${YARP_INCLUDE_DIRS}
)

# Source code for main executable.
FILE(GLOB folder_source 
	src/main.cpp
	src/AttentionSelectionModule.cpp
)
FILE(GLOB folder_header 
	include/iCub/AttentionSelectionInterfaces.h
	include/iCub/AttentionSelectionModule.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Source code for remote library
FILE(GLOB source_lib 
    src/RemoteAttentionSelection.cpp
)
FILE(GLOB header_lib
    include/iCub/RemoteAttentionSelection.h
    include/iCub/AttentionSelectionInterfaces.h
)
SOURCE_GROUP("Source Files" FILES ${source_lib})
SOURCE_GROUP("Header Files" FILES ${header_lib})

# Set up our main executable.
ADD_EXECUTABLE(${PROJECTNAME} 
	${folder_source} 
	${folder_header}
)
TARGET_LINK_LIBRARIES(${PROJECTNAME}
	${EGOSPHERELIB_LIBRARIES}
	${OpenCV_LIBRARIES}
	${YARP_LIBRARIES}
)
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

# Set up library
ADD_LIBRARY(attentionSelectionLib ${source_lib} ${header_lib})
TARGET_LINK_LIBRARIES(attentionSelectionLib)
INSTALL(TARGETS attentionSelectionLib DESTINATION lib)
