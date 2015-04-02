cmake_minimum_required(VERSION 2.6)

project(EFAA_Demo)

# RPATH settings
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH just yet
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#Add Yarp
find_package(YARP REQUIRED)
list(APPEND INCLUDES ${YARP_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})


# Add QT3
find_package(Qt3 REQUIRED)
ADD_DEFINITIONS(${QT_DEFINITIONS} )
ADD_DEFINITIONS(-DQT_DLL)
ADD_DEFINITIONS(-DQT_THREAD_SUPPORT)
ADD_DEFINITIONS(-Wall -O2)

#Add iCub
find_package(ICUB REQUIRED)
list(APPEND INCLUDES ${ICUB_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})


#Add OpenCV
find_package(OpenCV REQUIRED)
list(APPEND INCLUDES ${OpenCV_INCLUDE_DIRS})

#Set include directories for all targets
include_directories(${INCLUDES} ${QT_INCLUDE_DIR})

#Create variable for install path
set(INSTALL_PATH "${ICUB_DIR}/bin" CACHE PATH 
    "Installation directory for executables"
    )

add_subdirectory("src/sendBlobs")
add_subdirectory("src/iMultiTask")

