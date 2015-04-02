PROJECT(SimoxGrasping)


cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Simox_INCLUDE_DIR})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              
MESSAGE ("ICUB_LINK_FLAGS: ${ICUB_LINK_FLAGS}")
MESSAGE ("CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")

add_subdirectory(SimoxRobotViewer)
add_subdirectory(SimoxRobotViewerModule)
add_subdirectory(SimoxIkSolverModule)
add_subdirectory(SimoxGraspingPipelineControl)
add_subdirectory(SimoxHandTrackerModule)
add_subdirectory(SimoxHandEyeCalibrationGui)
add_subdirectory(SimoxMotionPlannerModule)
add_subdirectory(SimoxLegoLocalizerModule)
add_subdirectory(SimoxGraspExecutionModule)
