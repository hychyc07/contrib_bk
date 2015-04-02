#
# Author: Nikolaus Vahrenkamp
#

cmake_minimum_required(VERSION 2.6)
PROJECT(SimoxGrasping)

FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

include(iCubHelpers)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

				
ADD_SUBDIRECTORY(SimoxIkSolver)
ADD_SUBDIRECTORY(SimoxRobotViewer)
ADD_SUBDIRECTORY(SimoxGraspingPipeline)
ADD_SUBDIRECTORY(SimoxHandTrackerModule)
ADD_SUBDIRECTORY(SimoxMotionPlanner)
ADD_SUBDIRECTORY(SimoxHandEyeCalibrationGui)
ADD_SUBDIRECTORY(SimoxLegoLocalizer)
ADD_SUBDIRECTORY(SimoxGraspExecution)

## make all applications show up as single target
icub_app_all()



