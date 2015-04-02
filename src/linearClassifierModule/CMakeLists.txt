# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)
set(projectname linearClassifierModule)
project(${projectname})
include(FindLibSVMLin.cmake)
include(FindLibSVM.cmake)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include(iCubHelpers)
include(iCubOptions)

set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

IF(LIBSVM_FOUND)

	include_directories(${LIBSVM_INCLUDE_DIR})
	set(folder_header ${folder_header}
					  include/SVMNonLin.h)
					  
	set(folder_source ${folder_source}
					  src/SVMNonLin.cpp)					  
ENDIF()

IF(LIBSVMLIN_FOUND)
		
		
	set(folder_header ${folder_header}
					  include/SVMLinear.h)					  
	set(folder_source ${folder_source}
					  src/SVMLinear.cpp)	

	
	include_directories(${LIBSVMLIN_INCLUDE_DIR})
	



ENDIF()

if (LIBSVM_FOUND OR LIBSVMLIN_FOUND)
	set(folder_header ${folder_header}
					  include/linearClassifierThread.h
					  include/linearClassifierModule.h
					  include/win_dirent.h )
					 					  
	set(folder_source ${folder_source}
					  src/linearClassifierThread.cpp
					  src/linearClassifierModule.cpp
					  src/main.cpp )
					  
	source_group("Source Files" FILES ${folder_source})
	source_group("Header Files" FILES ${folder_header})
	include_directories(${PROJECT_SOURCE_DIR}/include)
	add_executable(${projectname} ${folder_source} ${folder_header} )
	
	
	
endif()

if (LIBSVM_FOUND AND LIBSVMLIN_FOUND)
	target_link_libraries(${projectname} ${YARP_LIBRARIES} ${LIBSVMLIN_LIBRARIES} ${LIBSVM_LIBRARIES})
	install(TARGETS ${projectname} DESTINATION ${ICUB_DIR}/bin)
elseif(LIBSVMLIN_FOUND)
	target_link_libraries(${projectname} ${YARP_LIBRARIES} ${LIBSVMLIN_LIBRARIES})
	install(TARGETS ${projectname} DESTINATION ${ICUB_DIR}/bin)
endif()

