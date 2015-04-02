#MESSAGE(STATUS "YarpVectorMixer Library")

FIND_LIBRARY(YarpVectorMixer_LIBRARIES YarpVectorMixer ${YarpVectorMixer_DIR}/lib)

IF (NOT YarpVectorMixer_LIBRARIES)
  SET(YarpVectorMixer_LIBRARIES YarpVectorMixer CACHE INTERNAL "YarpVectorMixer library")
  SET(YarpVectorMixer_LIBRARIES YarpVectorMixer)
ENDIF (NOT YarpVectorMixer_LIBRARIES)


SET(YarpVectorMixer_INCLUDE_DIR ${YarpVectorMixer_DIR}/include)

SET(YarpVectorMixer_DEPENDENCIES YARP)

