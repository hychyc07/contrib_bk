CMAKE_MINIMUM_REQUIRED(VERSION 2.6)			



set(SOURCES
    "dataassociator.cpp"
    "datacluster.cpp" 
    "dataelement3d.cpp" 
    "fuserthread.cpp" 
    "main.cpp"
    "targetcreationdeletion.cpp"
    "targetobject.cpp" 
    "timer.cpp"
    "validationmatrix.cpp"
    "headThread.cpp"
    "taskThread.cpp"
    "rightHandThread.cpp"
    )
    
include_directories(${INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Create everything needed to build our executable.
add_executable(iMultiTask ${SOURCES})

set(LIBS
    ${OpenCV_LIBRARIES}
    ${YARP_LIBRARIES}
    ${QT_LIBRARIES}
    icubmod
    )

target_link_libraries(iMultiTask ${LIBS} )

install(TARGETS iMultiTask DESTINATION ${INSTALL_PATH})
