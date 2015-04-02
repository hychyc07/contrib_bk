CMAKE_MINIMUM_REQUIRED(VERSION 2.6)			

#Compile cvBlob library
set(cvBlob_SOURCES
    "cvBlob/cvblob.cpp"
    "cvBlob/cvlabel.cpp"
    "cvBlob/cvaux.cpp"
    "cvBlob/cvcontour.cpp"
    "cvBlob/cvtrack.cpp"
    "cvBlob/cvcolor.cpp"
    )

add_library(cvblob ${cvBlob_SOURCES})


set(SOURCES
    "sendBlobs.cpp"
    )
    
# Need to refresh include directories to include cvBlob headers
include_directories(${INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/cvBlob")

# Create everything needed to build our executable.
add_executable(sendBlobs ${SOURCES})

set(LIBS
    ${OpenCV_LIBRARIES}
    ${YARP_LIBRARIES}
    cvblob
    )

target_link_libraries(sendBlobs ${LIBS} )

install(TARGETS sendBlobs DESTINATION ${INSTALL_PATH})
