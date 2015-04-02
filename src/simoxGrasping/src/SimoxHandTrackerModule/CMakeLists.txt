PROJECT(SimoxHandTrackerModule)

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

IF(folder_source)
    ADD_EXECUTABLE(SimoxHandTrackerModule
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(SimoxHandTrackerModule SimoxRobotViewer ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Simox_LIBRARIES})


install(TARGETS SimoxHandTrackerModule DESTINATION ${INSTALL_PATH_BIN})

