# Copyright: (C) 2010 RobotCub Consortium
# Authors: Eric Sauser
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf  ${ICUB_DIR}/conf)

SET(PM_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(MAINPROJECTS_LIST       LIBRARIES MODULES TOOLS GUIS)


SET(LIBRARIES_PATH          libraries)
SET(LIBRARIES_LIST          StdTools
                            MathLib
                            GLTools
                            YarpTools
)


SET(MODULES_PATH            modules)
SET(MODULES_LIST            iCubMotionControllerExt
                            iCubRecordAndReplay
                            GraspAdaptation
)

SET(TOOLS_PATH              tools)
SET(TOOLS_LIST              YarpVectorSplitter
                            YarpVectorMixer
                            YarpVectorBag
                            ResetSkinPart
                            FingertipPreprocessor
)

# see below to enable the guis
SET(GUIS_PATH               guis)
SET(GUIS_LIST               
)


SET(BUILD_ForceClosureCalculator  FALSE CACHE BOOL "Do you want to build <ForceClosureCalculator> (require the qhull library)")
IF(BUILD_ForceClosureCalculator)
    SET(TOOLS_LIST              ${TOOLS_LIST}
                                ForceClosureCalculator)
ENDIF(BUILD_ForceClosureCalculator)

SET(BUILD_GUIS  TRUE CACHE BOOL "Do you want to build the GUIS? (require Qt OpenGL and so forth)")
IF(BUILD_GUIS)
    SET(GUIS_LIST               iCubSkinGui3D
    )
ENDIF(BUILD_GUIS)






FOREACH(CURRPROJECT_NAME ${MAINPROJECTS_LIST})
    FOREACH(SUBPROJECT_NAME ${${CURRPROJECT_NAME}_LIST})
        SET(${SUBPROJECT_NAME}_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${${CURRPROJECT_NAME}_PATH}/${SUBPROJECT_NAME})
    ENDFOREACH(SUBPROJECT_NAME)
ENDFOREACH(CURRPROJECT_NAME ${MAINPROJECTS_LIST})


FOREACH(CURRPROJECT_NAME ${MAINPROJECTS_LIST})
    #SET(PM_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/${${CURRPROJECT_NAME}_PATH})
    FOREACH(SUBPROJECT_NAME ${${CURRPROJECT_NAME}_LIST})
        ADD_SUBDIRECTORY(${${CURRPROJECT_NAME}_PATH}/${SUBPROJECT_NAME})
    ENDFOREACH(SUBPROJECT_NAME)
ENDFOREACH(CURRPROJECT_NAME ${MAINPROJECTS_LIST})



