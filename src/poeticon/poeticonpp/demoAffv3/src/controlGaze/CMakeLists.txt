CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME controlGaze)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubHelpers)
#icub_install_with_rpath()

FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

# iCub applications of this contrib subfolder
# =====================================================================================

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

# Application 1
FILE(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
icub_app(${PROJECTNAME})
icub_app_install(${PROJECTNAME} FILES ${conf} DESTINATION conf)

# Collect all applications of this contrib subfolder and create a single build target
#icub_app_all()
