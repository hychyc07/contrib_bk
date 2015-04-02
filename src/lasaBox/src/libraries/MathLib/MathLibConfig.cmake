#MESSAGE(STATUS "MathLib Library")

FIND_LIBRARY(MathLib_LIBRARIES MathLib ${MathLib_DIR}/lib)

IF (NOT MathLib_LIBRARIES)
  SET(MathLib_LIBRARIES MathLib CACHE INTERNAL "MathLib library")
  SET(MathLib_LIBRARIES MathLib)
ENDIF (NOT MathLib_LIBRARIES)


SET(MathLib_INCLUDE_DIR ${MathLib_DIR}/include)

SET(MathLib_DEPENDENCIES )

