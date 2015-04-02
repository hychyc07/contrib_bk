# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak <www.martinpeniak.com>
# CopyPolicy: Released under the terms of the FreeBSD license.

find_package(CUDA REQUIRED) 

if(CUDA_FOUND)
	add_subdirectory(aquila-utility)
	add_subdirectory(aquila-nnet)
	add_subdirectory(aquila-image)
	message(STATUS "The following Aquila libraries will be compiled:")
	message(STATUS " - aquila-utility")
	message(STATUS " - aquila-nnet")
	message(STATUS " - aquila-image")
else()
	message("The following Aquila libraries will not be compiled:")
	message(" - aquila-utility")
	message(" - aquila-nnet")
	message(" - aquila-image")
endif()
