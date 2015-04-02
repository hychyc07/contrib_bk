cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME tactileGrasp)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_INSTALL_PREFIX "$ENV{ICUB_DIR}" CACHE PATH "Install path prefix, pre-appended onto install directory." FORCE)

# load support files
include(iCubHelpers)

# Add the example application "tactileGraspingDemo"
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
add_subdirectory(tactileGraspingDemo)
icub_app_all()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${skinDynLib_INCLUDE_DIRS})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/tactileGrasp/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
ADD_EXECUTABLE(${PROJECTNAME}  ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME}  ${YARP_LIBRARIES} skinDynLib)

if(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin/release)	
else(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin)
endif(WIN32)
