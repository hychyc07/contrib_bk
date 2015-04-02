IF(WIN32)
cmake_minimum_required(VERSION 2.8)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)


project(crawlPlannerMain)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
  
SET(COMPILE_CRAWL_SUPERVISOR_GUI FALSE CACHE BOOL "Do you want to build the optional gui to display the potiential field when planning ?")

add_subdirectory(crawlPlanner)
IF(COMPILE_CRAWL_SUPERVISOR_GUI)
	add_subdirectory(plannerSupervisor)
ENDIF(COMPILE_CRAWL_SUPERVISOR_GUI)
 
 
