#MESSAGE(STATUS "iCubSkinGui3D Library")

FIND_LIBRARY(iCubSkinGui3D_LIBRARIES iCubSkinGui3D ${iCubSkinGui3D_DIR}/lib)

IF (NOT iCubSkinGui3D_LIBRARIES)
  SET(iCubSkinGui3D_LIBRARIES iCubSkinGui3D CACHE INTERNAL "iCubSkinGui3D library")
  SET(iCubSkinGui3D_LIBRARIES iCubSkinGui3D)
ENDIF (NOT iCubSkinGui3D_LIBRARIES)


SET(iCubSkinGui3D_INCLUDE_DIR ${iCubSkinGui3D_DIR}/include)

SET(iCubSkinGui3D_DEPENDENCIES  GLTools YARP ICUB)
SET(iCubSkinGui3D_EXT_DEPENDENCIES Qt4)

