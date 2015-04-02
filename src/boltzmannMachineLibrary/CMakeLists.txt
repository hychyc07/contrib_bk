cmake_minimum_required(VERSION 2.8)
PROJECT (boltzmannMachineLibrary)
# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.


#################################

find_package(ICUB)
find_package(YARP)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

include(iCubOptions)
include(iCubHelpers)

find_package(IPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single applications
add_subdirectory(app)
add_subdirectory(src)

