CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME activeSeg)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

FIND_PACKAGE(OpenCV)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")
ADD_SUBDIRECTORY(app)

SET(USE_MULTI_THREADED FALSE CACHE BOOL "Select false if you do not have boost installed")

FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h include/iCub/*.inc)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
                    ${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
                    
					

# Create everything needed to build our executable.
IF (USE_MULTI_THREADED)
   ADD_EXECUTABLE( ${PROJECTNAME} ${folder_source} src/seg/textureGradient_multithreaded.cpp ${folder_header}) 
    ELSE()
   ADD_EXECUTABLE( ${PROJECTNAME} ${folder_source} src/seg/textureGradient.cpp ${folder_header}) 
ENDIF (USE_MULTI_THREADED)				

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${ICUB_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)


