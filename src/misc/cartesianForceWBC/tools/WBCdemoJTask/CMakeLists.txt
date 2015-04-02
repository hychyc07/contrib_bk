# Copyright (C) 2011 RBCS IIT
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 2.6)

set(KEYWORD "WBCdemoJTask")

# Start a project.
project(${KEYWORD})

include (CheckIncludeFileCXX)

if (NOT EIGEN2_DIR)
  set (EIGEN2_DIR $ENV{EIGEN2_DIR})
endif (NOT EIGEN2_DIR)
if (EIGEN2_DIR)
  message ("using EIGEN2_DIR ${EIGEN2_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
  include_directories (
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
else (EIGEN2_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    /usr/include/eigen2 /opt/local/include/eigen2)
  include_directories (/usr/include/eigen2 /opt/local/include/eigen2)
endif (EIGEN2_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
if (${HAVE_EIGEN2})
  message ("found Eigen")
else (${HAVE_EIGEN2})
  message (FATAL_ERROR "Eigen not found. Install it in a system location (e.g. `sudo apt-get install libeigen2-dev') or pass -DEIGEN2_DIR:path=/path/to/eigen2 to cmake.")
endif (${HAVE_EIGEN2})


#add_definitions (-DTIXML_USE_STL)
#add_definitions (-DTUTROB_XML_PATH_STR="${CMAKE_CURRENT_SOURCE_DIR}/app/conf/tutrob.icub-la-7dof.xml")
#add_subdirectory (
#  3rdparty/wbc_tinyxml/ ${CMAKE_BINARY_DIR}/3rdparty/wbc_tinyxml/
#  )

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

# Add the application "learningEndEffectorWrenchDemo"
#set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
#add_subdirectory(cartesianForceWBCDemo)
#icub_app_all()

# Search for source code.
file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/WBCdemo/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

# Okay if STANFORD_WBC is installed in linux, should do something about Windows...
link_directories(/usr/local/lib)

if (NOT STANFORD_WBC_DIR)
  set (STANFORD_WBC_DIR $ENV{STANFORD_WBC_DIR})
endif (NOT STANFORD_WBC_DIR)
if (STANFORD_WBC_DIR)
  message ("using STANFORD_WBC_DIR ${STANFORD_WBC_DIR}")
  include_directories ( ${STANFORD_WBC_DIR}/opspace/include )
  link_directories( ${STANFORD_WBC_DIR}/opspace ${STANFORD_WBC_DIR}/build/opspace)
else (STANFORD_WBC_DIR)
  message ("not using STANFORD_WBC_DIR")
endif (STANFORD_WBC_DIR)

# Set up our main executable.
if (folder_source)
  add_executable(${KEYWORD} ${folder_source} ${folder_header})
#  target_link_libraries(${KEYWORD} ${YARP_LIBRARIES} icubmod ctrlLib opspace jspace_test wbc_tinyxml)
  target_link_libraries(${KEYWORD} ${YARP_LIBRARIES} ctrlLib opspace jspace_test wbc_tinyxml)
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

