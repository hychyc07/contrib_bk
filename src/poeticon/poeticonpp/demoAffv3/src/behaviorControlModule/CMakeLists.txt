SET(KEYWORD "behaviorController")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

# Helper variable defining the root of the current project
SET(BEHAVIORCONTROL_SOURCE_DIR ${BEHAVIORCONTROL_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/behaviorControlModule)

# Check for AttentionSelection
set(ATTENTIONSELECTIONLIB_INCLUDE_DIRS ${ATTENTIONSELECTIONLIB_INCLUDE_DIRS} ${BEHAVIORCONTROL_SOURCE_DIR}/lib/attentionSystem/src/attentionSelection/include/iCub/*.h)
set(ATTENTIONSELECTIONLIB_LIBRARIES ${ATTENTIONSELECTIONLIB_LIBRARIES} ${CMAKE_SOURCE_DIR}/build/src/behaviorControlModule/lib/attentionSystem/src/attentionSelection/libattentionSelectionLib.a)


# Include the headers
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${ATTENTIONSELECTIONLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	src/*.cpp
)
FILE(GLOB folder_header 
	include/iCub/*.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
TARGET_LINK_LIBRARIES(${KEYWORD}
	${YARP_LIBRARIES}
	${ATTENTIONSELECTIONLIB_LIBRARIES}
)

