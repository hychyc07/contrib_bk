# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(executionModule)

FILE(GLOB folder_source *.cpp *.cc *.c)

SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_EXECUTABLE(executionModule ${folder_source})
TARGET_LINK_LIBRARIES(executionModule ${YARP_LIBRARIES})
INSTALL(TARGETS executionModule DESTINATION ${ICUB_DIR}/bin)

