cmake_minimum_required(VERSION 2.6)

PROJECT(portScope)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/conf)

# now load support files 
include(iCubHelpers)

# additional packages
FIND_PACKAGE(Qt3 REQUIRED)
FIND_PACKAGE(Qwt REQUIRED)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# applications
add_subdirectory(app)

#Link qtw for solving LQ minimization
#TARGET_LINK_LIBRARIES(portScope /home/iron/Code/qtw/lib/libqtw.a)
#INCLUDE_DIRECTORIES(/home/iron/Code/qtw/include)

#TARGET_LINK_LIBRARIES(portScope /media/WDBACKUP/Code/qtw/lib/libqtw.a)
#INCLUDE_DIRECTORIES(/media/WDBACKUP/Code/qtw/include)

ADD_DEFINITIONS(
    ${QT_DEFINITIONS}
)

#SET(QTW_LIBRARY /home/iron/Desktop/qwt-5.2.1/lib/libqwt.so)
#SET(QTW_INCLUDE_DIR /home/iron/Desktop/qwt-5.2.1/src)

#MESSAGE(${Qwt_LIBRARY})
ADD_EXECUTABLE(portScope ${folder_source} ${folder_header})
if (WIN32)
	INSTALL(TARGETS portScope DESTINATION ${ICUB_DIR}/bin/Release)
else()
	INSTALL(TARGETS portScope DESTINATION ${ICUB_DIR}/bin)
endif()
TARGET_LINK_LIBRARIES(portScope ${QT_LIBRARIES} ${YARP_LIBRARIES} ${Qwt_LIBRARY})
INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR} ${Qwt_INCLUDE_DIR} ${YARP_INCLUDE_DIRS})

