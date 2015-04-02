#MESSAGE(STATUS "YarpVectorBag Library")

FIND_LIBRARY(YarpVectorBag_LIBRARIES YarpVectorBag ${YarpVectorBag_DIR}/lib)

IF (NOT YarpVectorBag_LIBRARIES)
  SET(YarpVectorBag_LIBRARIES YarpVectorBag CACHE INTERNAL "YarpVectorBag library")
  SET(YarpVectorBag_LIBRARIES YarpVectorBag)
ENDIF (NOT YarpVectorBag_LIBRARIES)


SET(YarpVectorBag_INCLUDE_DIR ${YarpVectorBag_DIR}/include)

SET(YarpVectorBag_DEPENDENCIES YARP MathLib)

