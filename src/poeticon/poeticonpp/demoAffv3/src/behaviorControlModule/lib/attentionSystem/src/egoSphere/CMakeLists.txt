SET(KEYWORD "egoSphere")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubHelpers)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(KINEMATICS REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(SPMAP REQUIRED)
FIND_PACKAGE(SALIENCELIB REQUIRED) 

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
	${YARP_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
    ${KINEMATICS_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
    ${SPMAP_INCLUDE_DIRS}
    ${SALIENCELIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	src/main.cpp
	src/AcousticMap.cpp
	src/EgoSphereInterfaces.cpp
	src/EgoSphereModule.cpp
	src/Framerate.cpp
	src/IModalityMap.cpp
	src/ObjectMap.cpp
	src/SphereProjector.cpp
	src/VisualMap.cpp
)
FILE(GLOB folder_header 
	include/iCub/AcousticMap.h
	include/iCub/EgoSphereInterfaces.h
	include/iCub/EgoSphereModule.h
	include/iCub/Framerate.h
	include/iCub/IModalityMap.h
	include/iCub/ObjectMap.h
	include/iCub/SphereProjector.h
	include/iCub/VisualMap.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Source code for salience remote library
FILE(GLOB source_lib 
    src/RemoteEgoSphere.cpp
)
FILE(GLOB header_lib
    include/iCub/RemoteEgoSphere.h
    include/iCub/EgoSphereInterfaces.h
)
SOURCE_GROUP("Source Files" FILES ${source_lib})
SOURCE_GROUP("Header Files" FILES ${header_lib})


# Set up our main executable.
IF (folder_source)
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

# Set up library
ADD_LIBRARY(egoSphereLib ${source_lib} ${header_lib})
TARGET_LINK_LIBRARIES(
			egoSphereLib
			libgslcblas.a
)
INSTALL_TARGETS(/lib egoSphereLib)

