#MESSAGE(STATUS "YarpVectorSplitter Library")

FIND_LIBRARY(YarpVectorSplitter_LIBRARIES YarpVectorSplitter ${YarpVectorSplitter_DIR}/lib)

IF (NOT YarpVectorSplitter_LIBRARIES)
  SET(YarpVectorSplitter_LIBRARIES YarpVectorSplitter CACHE INTERNAL "YarpVectorSplitter library")
  SET(YarpVectorSplitter_LIBRARIES YarpVectorSplitter)
ENDIF (NOT YarpVectorSplitter_LIBRARIES)


SET(YarpVectorSplitter_INCLUDE_DIR ${YarpVectorSplitter_DIR}/include)

SET(YarpVectorSplitter_DEPENDENCIES YARP)

