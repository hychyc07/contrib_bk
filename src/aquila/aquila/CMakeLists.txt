# Copyright: 2012 RobotCub Consortium
# Author: Martin Peniak
# CopyPolicy: Released under the terms of the FreeBSD license

set(PROJECTNAME aquila)
project(${PROJECTNAME})

file(GLOB SOURCES src/*.cpp src/*.cc src/*.c)
file(GLOB HEADERS include/*.h)
file(GLOB UIS ui/*.ui)
file(GLOB RESOURCES res/*.qrc)

source_group("Sources" FILES ${SOURCES})
source_group("Headers" FILES ${HEADERS})
source_group("UIs" FILES ${UIS})
source_group("Resources" FILES ${RESOURCES})
source_group("Generated" REGULAR_EXPRESSION "(moc_.*cxx|ui_.*h|vbtest_.*cpp)$")

include_directories(${PROJECT_SOURCE_DIR}/include
					${QT_INCLUDES} 
					${OPENGL_INCLUDE_DIR}
                    			${YARP_INCLUDE_DIRS}
					${CMAKE_CURRENT_BINARY_DIR}
					${CMAKE_SOURCE_DIR}/src})

set(QT_USE_QTNETWORK TRUE)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

qt4_add_resources(RESOURCES ${RESOURCES})
qt4_wrap_cpp(MOC_SOURCES ${HEADERS})
qt4_wrap_ui(UIS ${UIS})

add_executable(${PROJECTNAME}
			   ${SOURCES}
			   ${HEADERS}
			   ${MOC_SOURCES}
			   ${RESOURCES}
			   ${UIS})



target_link_libraries(${PROJECTNAME}
					 ${QT_LIBRARIES}
					 ${OPENGL_LIBRARIES}
					 ${YARP_LIBRARIES})

if(WIN32)
    message(STATUS "\nWIN32 detected - linking ole32...")
    target_link_libraries(${PROJECTNAME} ole32)
endif(WIN32)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
