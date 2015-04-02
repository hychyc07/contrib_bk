# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini, Vadim Tikhanoff, Carlo Ciliberto, Stephane Lallee, Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project(interactiveObjectsLearning)

add_subdirectory(blobExtractor)
add_subdirectory(blobSelector)
add_subdirectory(milClassifier)
add_subdirectory(iolRADHelper)
add_subdirectory(iolStateMachineHandler)
add_subdirectory(ScSPMClassifier)
add_subdirectory(app)

