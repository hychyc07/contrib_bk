#MESSAGE(STATUS "iCubRecordAndReplay Library")

FIND_LIBRARY(iCubRecordAndReplay_LIBRARIES iCubRecordAndReplay ${iCubRecordAndReplay_DIR}/lib)

IF (NOT iCubRecordAndReplay_LIBRARIES)
  SET(iCubRecordAndReplay_LIBRARIES iCubRecordAndReplay CACHE INTERNAL "iCubRecordAndReplay library")
  SET(iCubRecordAndReplay_LIBRARIES iCubRecordAndReplay)
ENDIF (NOT iCubRecordAndReplay_LIBRARIES)


SET(iCubRecordAndReplay_INCLUDE_DIR ${iCubRecordAndReplay_DIR}/include)

SET(iCubRecordAndReplay_DEPENDENCIES MathLib YARP ICUB)

