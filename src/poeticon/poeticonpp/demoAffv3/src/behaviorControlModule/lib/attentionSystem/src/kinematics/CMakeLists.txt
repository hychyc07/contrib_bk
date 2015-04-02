SET(KEYWORD "kinematics")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubHelpers)

FIND_PACKAGE(GSL REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/kinematics/*.h include/iCub/head/*.h include/iCub/arm/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${GSL_INCLUDE_DIR}	
	${YARP_INCLUDE_DIRS}
)

LINK_LIBRARIES(
    ${GSL_LIBRARIES}
    ${GSLCBLAS_LIBRARIES}	
	${YARP_LIBRARIES}
)

# Set up our main executable.
IF (folder_source)
  ADD_LIBRARY(${KEYWORD} ${folder_source} ${folder_header})
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(/lib ${KEYWORD})


 
