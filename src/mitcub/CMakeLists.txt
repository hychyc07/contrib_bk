



cmake_minimum_required(VERSION 2.6)

project(MITCUB)

find_package(YARP)
link_libraries(${YARP_LIBRARIES})
include_directories(${YARP_INCLUDE_DIRS})
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

find_package(ICUB)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
include(iCubHelpers)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(INSTALL_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH "Absolute path to a folder where the compiled binary will be installed")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

# The following libraries are mandatory
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

set(USE_GURLS OFF CACHE BOOL "If true, compile GURLS package")

if(USE_GURLS)
	enable_language( Fortran )
	find_package(BLAS)
	find_package(LAPACK)
endif(USE_GURLS)

# Set the following variable to ON if you want to enable the use of OpenCV
set(USE_OPENCV OFF CACHE BOOL "Enables the compilation of the modules and examples that rely on the OpenCV library")
IF(USE_OPENCV)
  find_package(OpenCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
ENDIF(USE_OPENCV)

set(USE_SUPERVECTORS OFF CACHE BOOL "If TRUE the module to compute the supervectors will be used. Warning: require the library SIFTGPU!")
if(USE_SUPERVECTORS)
  find_package(SIFTGPU)
endif(USE_SUPERVECTORS)

#set(HMINLIBRARY hmin++)

if(USE_GURLS)
	add_subdirectory(gurls)
endif(USE_GURLS)


if(USE_OPENCV)
	add_subdirectory(hmin)
	add_subdirectory(supervectors)
	add_subdirectory(featureRepresentations)
	#add_subdirectory(featureRepresentationsTmp)
	add_subdirectory(modules)
endif(USE_OPENCV)