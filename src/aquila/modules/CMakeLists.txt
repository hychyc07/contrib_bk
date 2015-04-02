# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak <www.martinpeniak.com>
# CopyPolicy: Released under the terms of the FreeBSD license.

find_package(CUDA REQUIRED) 
find_package(OpenMP REQUIRED)

if(CUDA_FOUND AND OPENMP_FOUND)
	add_subdirectory(mtrnn)
	add_subdirectory(som)
	add_subdirectory(tracker)
	add_subdirectory(esn)
	add_subdirectory(era)
	add_subdirectory(altair)
	# ModuleGenerator:write:5
	message(STATUS "The following Aquila modules will be compiled:")
	message(STATUS " - mtrnn")
	message(STATUS " - som")
	message(STATUS " - tracker")
	message(STATUS " - esn")
	message(STATUS " - era")
	message(STATUS " - altair")
	# ModuleGenerator:write:6
else()
	message("The following Aquila modules will not be compiled:")
	message(" - mtrnn")
	message(" - som")
	message(" - tracker")
	message(" - esn")
	message(" - era")
	message(" - altair")
	# ModuleGenerator:write:7
endif()
