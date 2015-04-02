# Copyright (C) 2011 RBCS IIT
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 2.6)

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

find_package(YARP REQUIRED)

add_definitions (-DTIXML_USE_STL)
add_definitions (-DTUTROB_XML_PATH_STR5="${CMAKE_CURRENT_SOURCE_DIR}/../../app/conf/tutrob.icub-la-emu7dof-wo-15deg.xml")
add_subdirectory (
  ../../3rdparty/wbc_tinyxml/ ${CMAKE_BINARY_DIR}/3rdparty/wbc_tinyxml/
  )

include_directories (
  include
  ${YARP_INCLUDE_DIRS}
  )

set (FLTK_SKIP_OPENGL ON)
set (FLTK_SKIP_FORMS ON)
set (FLTK_SKIP_IMAGES ON)
set (FLTK_SKIP_FLUID ON)
include (FindFLTK)

# Okay if STANFORD_WBC is installed in linux, should do something about Windows...
link_directories(/usr/local/lib)

add_library (tut7 STATIC src/tutsim7.cpp)
add_executable (sim7 src/main7.cpp)
target_link_libraries (sim7 tut7 jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

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

option(ENABLE_experimental "Choose if you want to compile experimental modules" FALSE)
IF (ENABLE_experimental)
    add_library (tut5 STATIC src/tutsim5.cpp)
    add_executable (sim5 src/main5.cpp)
    target_link_libraries (sim5 tut5 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

add_executable (sim7_jtask src/main7_jtask.cpp)
target_link_libraries (sim7_jtask tut7 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

add_executable (sim7_gravity src/main7_gravity.cpp)
target_link_libraries (sim7_gravity tut7 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

add_executable (sim7_gravity2 src/main7_gravity2.cpp)
target_link_libraries (sim7_gravity2 tut7 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

add_executable (sim7_opspace src/main7_opspace.cpp)
target_link_libraries (sim7_opspace tut7 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

add_executable (sim7_eepos src/main7_eepos.cpp)
target_link_libraries (sim7_eepos tut7 opspace jspace_test wbc_tinyxml ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

#add_executable (test7 src/test7.cpp)
#target_link_libraries (test7 tut7 opspace jspace_test wbc_tinyxml ctrlLib ${FLTK_LIBRARIES} ${YARP_LIBRARIES})

ENDIF (ENABLE_experimental)

