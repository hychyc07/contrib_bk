# Find Qwt
# ~~~~~~~~
# Copyright (c) 2010, Tim Sutton <tim at linfiniti.com>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
# Once run this will define: 
# 
# Qwt_FOUND       = system has Qwt lib
#
# Qwt_LIBRARY     = full path to the Qwt library
#
# Qwt_INCLUDE_DIR      = where to find headers 
#

set(Qwt_DIR $ENV{Qwt_DIR})

# MESSAGE("Searching for Qwt in " ${Qwt_DIR})

FIND_PATH(Qwt_INCLUDE_DIR qwt.h 
  PATHS /usr/include
  /usr/local/include
  ${Qwt_DIR}/src
  ${Qwt_DIR}/include 
  PATH_SUFFIXES qwt-qt3 qwt
  )
FIND_LIBRARY(Qwt_LIBRARY NAMES libqwt qwt qwt521 PATHS 
  /usr/lib
  /usr/local/lib
  ${Qwt_DIR}/lib
  )
IF (NOT Qwt_LIBRARY)
  # try using ubuntu lib naming
  FIND_LIBRARY(Qwt_LIBRARY qwt-qt3 PATHS 
    /usr/lib
    /usr/local/lib
    ${Qwt_DIR}/lib
    )
ENDIF (NOT Qwt_LIBRARY)

IF (Qwt_INCLUDE_DIR AND Qwt_LIBRARY)
  SET(Qwt_FOUND TRUE)
ENDIF (Qwt_INCLUDE_DIR AND Qwt_LIBRARY)

IF (Qwt_FOUND)
  IF (NOT Qwt_FIND_QUIETLY)
    MESSAGE(STATUS "Found Qwt: ${Qwt_LIBRARY}")
  ENDIF (NOT Qwt_FIND_QUIETLY)
ELSE (Qwt_FOUND)
  IF (Qwt_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find Qwt")
  ENDIF (Qwt_FIND_REQUIRED)
ENDIF (Qwt_FOUND)
