cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME piSquare)
project(${PROJECTNAME})

#find packages
find_package(YARP)
find_package(ICUB)
find_package(Boost REQUIRED)
 
#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

#load some macros useful for applications (see below)
include(iCubHelpers)

# Add the example application "piSquareDemo"
#set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
#add_subdirectory(piSquareDemo)
#icub_app_all()

# add local directory and other (optional) packages to to search path for header files
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories($ENV{EIGEN_INCLUDE_PATH})

# Search for source code.
FILE(GLOB folder_source src/dmpMotionGeneration/*.cpp 
			src/locallyWeightedRegression/*.cpp
			src/policyImprovement/*.cpp
			src/policyImprovementManager/*.cpp
			src/policyLibrary/*.cpp)

FILE(GLOB folder_header	include/iCub/piSquare/dmpMotionGeneration/*.h
			include/iCub/piSquare/locallyWeightedRegression/*.h
			include/iCub/piSquare/policyImprovement/*.h
			include/iCub/piSquare/policyImprovementManager/*.h
			include/iCub/piSquare/policyLibrary/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(	${PROJECTNAME}
			${YARP_LIBRARIES} 
			${Boost_LIBRARIES}
		     )

