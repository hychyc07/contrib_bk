PROJECT(SimoxHandEyeCalibrationGui)

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

################################## moc'ing ##############################
set(GUI_MOC_HDRS
    ${PROJECT_SOURCE_DIR}/SimoxHandEyeCalibrationWindow.h
)

set(GUI_UIS
    ${PROJECT_SOURCE_DIR}/SimoxHandEyeCalibration.ui
)

#MESSAGE("GUI_MOC_HDRS: ${GUI_MOC_HDRS}")
#MESSAGE("GUI_UIS: ${GUI_UIS}")
qt4_wrap_cpp(folder_source ${GUI_MOC_HDRS})
qt4_wrap_ui(UI_HEADER ${GUI_UIS})
get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
list(APPEND folder_header ${UI_HEADER})
#MESSAGE("folder_source: ${folder_source}")
#MESSAGE("folder_header: ${folder_header}")

include_directories(${UI_HEADER_DIR})



IF(folder_source)
    ADD_EXECUTABLE(SimoxHandEyeCalibrationGui
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(SimoxHandEyeCalibrationGui ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Simox_LIBRARIES})

install(TARGETS SimoxHandEyeCalibrationGui DESTINATION ${INSTALL_PATH_BIN})
