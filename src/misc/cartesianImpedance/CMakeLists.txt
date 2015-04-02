############################## AUTO GENERATED CMAKELISTS ##############################
 
cmake_minimum_required(VERSION 2.6)
################## Setting up Poject ########################
################## Add Project Dependencies Here ############
 
SET(V_PROJECT_NAME cartesianImpedance)
SET(V_PROJECT_DEPS YARP ICUB)
SET(V_EXT_LIB_DEPS )
SET(V_INCLUDE_BASE include)
SET(V_BIN_BASE bin)
SET(V_SRC_BASE src)
Project(${V_PROJECT_NAME})
############################################################
 
 
########################## Processing Include Directories ###############################
SET(V_INCLUDE_LIST ${CMAKE_CURRENT_SOURCE_DIR}/${V_INCLUDE_BASE})
SET(V_LIBRARY )
SET(V_LIBRARIES )
FOREACH(DEP ${V_PROJECT_DEPS})
	FIND_PACKAGE(${DEP})
		IF(NOT ${${DEP}_FOUND})
		FIND_PACKAGE(${DEP}  PATHS  ${CMAKE_ROOT}/Modules ${CMAKE_SOURCE_DIR} ${RobotToolKit_core_DIR})
	ENDIF()
	LIST(APPEND V_INCLUDE_LIST ${${DEP}_INCLUDES} ${${DEP}_INCLUDE_DIR} ${${DEP}_INCLUDE_DIRS} ${${DEP}_INCLUDE_DIRECTORIES})
	LIST(APPEND V_LIBRARIES ${${DEP}_LIBRARY} ${${DEP}_LIBRARIES})
ENDFOREACH(DEP)
INCLUDE_DIRECTORIES(${V_INCLUDE_LIST})
########################################################################################
 
 
################################### Adding the Executable #################################
SET(EXECUTABLE_OUTPUT_PATH ${V_BIN_BASE})
FILE(GLOB V_SOURCEFILES ${V_SRC_BASE}/*.c ${V_SRC_BASE}/*.cpp ${V_SRC_BASE}/*.h ${V_SRC_BASE}/*.hpp)
ADD_EXECUTABLE(${V_PROJECT_NAME} ${V_SOURCEFILES})
########################################################################################
 
 
################################### Linking Libraries #################################
FOREACH(DEP_NAME ${V_EXT_LIB_DEPS})
    LIST(APPEND V_LIBRARIES ${DEP_NAME})
ENDFOREACH(DEP_NAME)
IF(V_LIBRARIES)
	LIST(REMOVE_DUPLICATES V_LIBRARIES)
ENDIF(V_LIBRARIES)
TARGET_LINK_LIBRARIES(${V_PROJECT_NAME} ${V_LIBRARIES})
########################################################################################
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${V_PROJECT_NAME}.in ${CMAKE_CURRENT_SOURCE_DIR}/${V_PROJECT_NAME}Config.cmake @ONLY)
INSTALL(TARGETS ${V_PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin)
