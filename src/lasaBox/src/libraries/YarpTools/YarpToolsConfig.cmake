#MESSAGE(STATUS "YarpTools Library")

FIND_LIBRARY(YarpTools_LIBRARIES YarpTools ${YarpTools_DIR}/lib)

IF (NOT YarpTools_LIBRARIES)
  SET(YarpTools_LIBRARIES YarpTools CACHE INTERNAL "YarpTools library")
  SET(YarpTools_LIBRARIES YarpTools)
ENDIF (NOT YarpTools_LIBRARIES)


SET(YarpTools_INCLUDE_DIR ${YarpTools_DIR}/include)

SET(YarpTools_DEPENDENCIES YARP StdTools MathLib)

