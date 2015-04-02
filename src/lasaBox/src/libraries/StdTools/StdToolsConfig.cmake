#MESSAGE(STATUS "StdTools Library")

FIND_LIBRARY(StdTools_LIBRARIES StdTools ${StdTools_DIR}/lib)

IF (NOT StdTools_LIBRARIES)
  SET(StdTools_LIBRARIES StdTools CACHE INTERNAL "StdTools library")
  SET(StdTools_LIBRARIES StdTools)
ENDIF (NOT StdTools_LIBRARIES)


SET(StdTools_INCLUDE_DIR ${StdTools_DIR}/include)

SET(StdTools_DEPENDENCIES )

