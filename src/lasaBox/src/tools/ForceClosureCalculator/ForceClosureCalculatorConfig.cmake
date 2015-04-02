#MESSAGE(STATUS "ForceClosureCalculator Library")

FIND_LIBRARY(ForceClosureCalculator_LIBRARIES ForceClosureCalculator ${ForceClosureCalculator_DIR}/lib)

IF (NOT ForceClosureCalculator_LIBRARIES)
  SET(ForceClosureCalculator_LIBRARIES ForceClosureCalculator CACHE INTERNAL "ForceClosureCalculator library")
  SET(ForceClosureCalculator_LIBRARIES ForceClosureCalculator)
ENDIF (NOT ForceClosureCalculator_LIBRARIES)


SET(ForceClosureCalculator_INCLUDE_DIR ${ForceClosureCalculator_DIR}/include)

SET(ForceClosureCalculator_DEPENDENCIES MathLib)
SET(ForceClosureCalculator_INCDIR_DEPENDENCIES  ./lib/include)
SET(ForceClosureCalculator_LIBS_DEPENDENCIES    qhull)
SET(ForceClosureCalculator_LIBDIR_DEPENDENCIES  ./lib)

