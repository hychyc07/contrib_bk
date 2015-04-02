# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Try to locate the OpenTLD library
#
# User is required to set the following environment variables
# OpenTLD_ROOT => the root directory 
# OpenTLD_DIR  => the build directory
#
# Create the following variables:
# OpenTLD_INCLUDE_DIRS
# OpenTLD_LIBRARIES
# OpenTLD_FOUND

set(OpenTLD_ROOT $ENV{OpenTLD_ROOT} CACHE PATH "Path to OpenTLD source directory")
set(OpenTLD_DIR  $ENV{OpenTLD_DIR}  CACHE PATH "Path to OpenTLD build directory")

if(OpenTLD_ROOT AND OpenTLD_DIR)
   set(OpenTLD_INCLUDE_DIRS ${OpenTLD_ROOT}/src/3rdparty/cvblobs
                            ${OpenTLD_ROOT}/src/libopentld/tld
                            ${OpenTLD_ROOT}/src/libopentld/mftracker
                            ${OpenTLD_ROOT}/src/libopentld/imacq)

   find_library(OpenTLD_OpenTLD_LIB NAMES opentld  PATHS ${OpenTLD_DIR}/lib)
   find_library(OpenTLD_CVBLOBS_LIB NAMES cvblobs  PATHS ${OpenTLD_DIR}/lib)
   find_library(OpenTLD_CONFIG_LIB  NAMES config++ PATHS ${OpenTLD_DIR}/lib)

   mark_as_advanced(OpenTLD_OpenTLD_LIB)
   mark_as_advanced(OpenTLD_CVBLOBS_LIB)
   mark_as_advanced(OpenTLD_CONFIG_LIB)

   if(UNIX)
      execute_process(COMMAND uname -m         OUTPUT_VARIABLE MACHINE_TYPE)
      execute_process(COMMAND gcc -dumpversion OUTPUT_VARIABLE GCC_VER)
      string(REPLACE "\n" "" MACHINE_TYPE ${MACHINE_TYPE})
      string(REPLACE "\n" "" GCC_VER ${GCC_VER})

      find_library(OpenTLD_GOMP_LIB gomp /usr/lib/gcc/${MACHINE_TYPE}-linux-gnu/${GCC_VER})
      mark_as_advanced(OpenTLD_GOMP_LIB)
      if(NOT OpenTLD_GOMP_LIB)
          message(STATUS "OpenMP is required to use OpenTLD on linux")
      endif()      

      if(OpenTLD_OpenTLD_LIB AND OpenTLD_CVBLOBS_LIB AND OpenTLD_CONFIG_LIB AND OpenTLD_GOMP_LIB)
         set(OpenTLD_LIBRARIES ${OpenTLD_OpenTLD_LIB} ${OpenTLD_CVBLOBS_LIB} ${OpenTLD_CONFIG_LIB} ${OpenTLD_GOMP_LIB})
         set(OpenTLD_FOUND TRUE)
      else()
         set(OpenTLD_INCLUDE_DIRS "")
         set(OpenTLD_LIBRARIES    "")
         set(OpenTLD_FOUND FALSE)
      endif()
   else()
      if(OpenTLD_OpenTLD_LIB AND OpenTLD_CVBLOBS_LIB AND OpenTLD_CONFIG_LIB)
         set(OpenTLD_LIBRARIES ${OpenTLD_OpenTLD_LIB} ${OpenTLD_CVBLOBS_LIB} ${OpenTLD_CONFIG_LIB})
         set(OpenTLD_FOUND TRUE)
      else()
         set(OpenTLD_INCLUDE_DIRS "")
         set(OpenTLD_LIBRARIES    "")
         set(OpenTLD_FOUND FALSE)
      endif()
   endif()
else()
   set(OpenTLD_FOUND FALSE)
endif()

set(OpenTLD_ROOT "")
set(OpenTLD_DIR  "")


