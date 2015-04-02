#MESSAGE(STATUS "ResetSkinPart Library")

FIND_LIBRARY(ResetSkinPart_LIBRARIES ResetSkinPart ${ResetSkinPart_DIR}/lib)

IF (NOT ResetSkinPart_LIBRARIES)
  SET(ResetSkinPart_LIBRARIES ResetSkinPart CACHE INTERNAL "ResetSkinPart library")
  SET(ResetSkinPart_LIBRARIES ResetSkinPart)
ENDIF (NOT ResetSkinPart_LIBRARIES)


SET(ResetSkinPart_INCLUDE_DIR ${ResetSkinPart_DIR}/include)

SET(ResetSkinPart_DEPENDENCIES YARP ICUB)

