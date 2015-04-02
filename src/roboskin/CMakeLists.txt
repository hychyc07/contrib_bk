cmake_minimum_required(VERSION 2.8)
find_package(YARP REQUIRED)

set(PROJECTNAME roboskin)
PROJECT(${PROJECTNAME})

add_subdirectory(skinForceControlLib)
add_subdirectory(skinForceControl)
add_subdirectory(skinCalibrationPlanner)
add_subdirectory(skinCalibration)
add_subdirectory(safeReach)

icub_app_all()
