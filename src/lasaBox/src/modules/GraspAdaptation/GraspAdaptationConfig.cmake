#MESSAGE(STATUS "GraspAdaptation Library")

FIND_LIBRARY(GraspAdaptation_LIBRARIES GraspAdaptation ${GraspAdaptation_DIR}/lib)

IF (NOT GraspAdaptation_LIBRARIES)
  SET(GraspAdaptation_LIBRARIES GraspAdaptation CACHE INTERNAL "GraspAdaptation library")
  SET(GraspAdaptation_LIBRARIES GraspAdaptation)
ENDIF (NOT GraspAdaptation_LIBRARIES)


SET(GraspAdaptation_INCLUDE_DIR ${GraspAdaptation_DIR}/include)

SET(GraspAdaptation_DEPENDENCIES YARP ICUB MathLib)

