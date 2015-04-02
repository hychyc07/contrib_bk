PROJECT(SimoxLegoLocalizerModule)

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
    ADD_EXECUTABLE(SimoxLegoLocalizerModule
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

#MESSAGE ("ICUB_LIBRARIES: ${ICUB_LIBRARIES} ")
#MESSAGE ("LIBRARIES: ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${SIMOX_LIBRARIES} ${SIMOX_VISUALIZATION_LIBS} ")

#MESSAGE("SIMOX_LIBRARIES:${SIMOX_LIBRARIES}")

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(SimoxLegoLocalizerModule SimoxRobotViewer ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Simox_LIBRARIES})

#MESSAGE("YARP_LIBRARIES: ${YARP_LIBRARIES}")
#MESSAGE("ICUB_LIBRARIES: ${ICUB_LIBRARIES}")
#MESSAGE("SIMOX_LIBRARIES: ${SIMOX_LIBRARIES}")
#MESSAGE("SIMOX_VISUALIZATION_LIBS: ${SIMOX_VISUALIZATION_LIBS}")
#MESSAGE("TARGET_LINK_LIBRARIES: ${LINK_FLAGS}")
install(TARGETS SimoxLegoLocalizerModule DESTINATION ${INSTALL_PATH_BIN})

