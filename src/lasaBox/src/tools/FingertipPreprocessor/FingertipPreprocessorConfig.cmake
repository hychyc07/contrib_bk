#MESSAGE(STATUS "FingertipPreprocessor Library")

FIND_LIBRARY(FingertipPreprocessor_LIBRARIES FingertipPreprocessor ${FingertipPreprocessor_DIR}/lib)

IF (NOT FingertipPreprocessor_LIBRARIES)
  SET(FingertipPreprocessor_LIBRARIES FingertipPreprocessor CACHE INTERNAL "FingertipPreprocessor library")
  SET(FingertipPreprocessor_LIBRARIES FingertipPreprocessor)
ENDIF (NOT FingertipPreprocessor_LIBRARIES)


SET(FingertipPreprocessor_INCLUDE_DIR ${FingertipPreprocessor_DIR}/include)

SET(FingertipPreprocessor_DEPENDENCIES MathLib YARP ICUB)

