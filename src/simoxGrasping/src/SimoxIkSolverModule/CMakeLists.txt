PROJECT(SimoxIkSolverModule)

INCLUDE (CheckIncludeFiles)

cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# set rpath in installed binary file
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

## experimental
#SET (manipulability_header "/usr/local/src/robot/niko/ManipulabilityAnalysis/Manipulability.h")
#SET (manipulability_src "/usr/local/src/robot/niko/ManipulabilityAnalysis/Manipulability.cpp")
#SET (manipulability2_header "/usr/local/src/robot/niko/IKQuality/PoseQualityExtendedManipulability.h")
#SET (manipulability2_src "/usr/local/src/robot/niko/IKQuality/PoseQualityExtendedManipulability.cpp")

#SET (manipulability_header "${VirtualRobot_INCLUDE_DIR}/Workspace/Manipulability.h")
#SET (manipulability_src "C:/Projects/IIT_Projects/ManipulabilityAnalysis/Manipulability.cpp")
#SET (manipulability2_header "C:/Projects/IIT_Projects/IKQuality/PoseQualityExtendedManipulability.h")
#SET (manipulability2_src "C:/Projects/IIT_Projects/IKQuality/PoseQualityExtendedManipulability.cpp")
#MESSAGE ("Looking for ${manipulability_header}")

#CHECK_INCLUDE_FILES(manipulability_header EXTENDED_MANIPULABILITY_MEASURE)
#MESSAGE ("EXTENDED_MANIPULABILITY_MEASURE:${EXTENDED_MANIPULABILITY_MEASURE}")
#IF (EXTENDED_MANIPULABILITY_MEASURE)
#IF (EXISTS ${manipulability_header})
#	IF (EXISTS ${manipulability2_header})
		MESSAGE ("USING EXTENDED MANIPULABILITY MEASURE")
		ADD_DEFINITIONS(-DEXTENDED_MANIPULABILITY_MEASURE)
#		SET(folder_source ${folder_source} ${manipulability_src} ${manipulability2_src})
#		SET(folder_header ${folder_header} ${manipulability_header} ${manipulability2_header})
#	ELSE()
#		MESSAGE ("NOT USING EXTENDED MANIPULABILITY MEASURE2")
#	ENDIF()
#ELSE()
#	MESSAGE ("NOT USING EXTENDED MANIPULABILITY MEASURE1")
#ENDIF()

IF(folder_source)
    ADD_EXECUTABLE(SimoxIkSolverModule
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)




# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(SimoxIkSolverModule SimoxRobotViewer ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Simox_LIBRARIES})


install(TARGETS SimoxIkSolverModule DESTINATION ${INSTALL_PATH_BIN})

