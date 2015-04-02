# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME poeticonpp)
project(${PROJECTNAME})

add_subdirectory(approximateSensing)
add_subdirectory(blobDescriptorModule)
add_subdirectory(demoGraspManager)
add_subdirectory(edisonSegmentationModule)
add_subdirectory(faceDetector)
add_subdirectory(nearestObjDetector)
add_subdirectory(sequentialLabeller)
add_subdirectory(wonderAround)
add_subdirectory(app)
