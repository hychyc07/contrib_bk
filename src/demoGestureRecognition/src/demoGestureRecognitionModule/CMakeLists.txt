# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)
set(projectname demoGestureRecognitionModule)
project(${projectname})

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(${projectname} ${source} ${header} )
target_link_libraries(${projectname} ${YARP_LIBRARIES})
install(TARGETS ${projectname} DESTINATION ${ICUB_DIR}/bin)

