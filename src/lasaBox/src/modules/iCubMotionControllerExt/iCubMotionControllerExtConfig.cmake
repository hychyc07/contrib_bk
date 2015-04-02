#MESSAGE(STATUS "iCubMotionControllerExt Library")

FIND_LIBRARY(iCubMotionControllerExt_LIBRARIES iCubMotionControllerExt ${iCubMotionControllerExt_DIR}/lib)

IF (NOT iCubMotionControllerExt_LIBRARIES)
  SET(iCubMotionControllerExt_LIBRARIES iCubMotionControllerExt CACHE INTERNAL "iCubMotionControllerExt library")
  SET(iCubMotionControllerExt_LIBRARIES iCubMotionControllerExt)
ENDIF (NOT iCubMotionControllerExt_LIBRARIES)


SET(iCubMotionControllerExt_INCLUDE_DIR ${iCubMotionControllerExt_DIR}/include)

SET(iCubMotionControllerExt_DEPENDENCIES YARP ICUB MathLib)

