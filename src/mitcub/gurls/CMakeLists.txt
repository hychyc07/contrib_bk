#
# Copyright (C) 2012 Matteo Santoro
# CopyPolicy: Released under the terms of GPL
#

project(gurls++)

set(GURLSLIBRARY gurls++)
set(SRCDIR src/)
set(HDRDIR include/)
set(DEMODIR demo/)
set(TESTDIR test/)
set(MISCDIR misc/)


function(add_all_executables ROOTDIR SRCDIR LINK_LIBS)

    file(GLOB sources RELATIVE ${ROOTDIR} ${SRCDIR}*.cpp)

    set(executables "")

    foreach(source_file ${sources})

        string(REGEX REPLACE ".cpp" "" executable ${source_file})
        string(REGEX REPLACE ${SRCDIR} "" executable ${executable})

        set(executables ${executables} ${executable})

        add_executable(${executable} ${source_file} )
        target_link_libraries(${executable} ${LINK_LIBS})

        set_property(TARGET ${executable} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
        set_property(TARGET ${executable} PROPERTY INSTALL_RPATH ${INSTALL_RPATH} ${CMAKE_INSTALL_PREFIX}/lib)

    endforeach(source_file)


    INSTALL(TARGETS ${executables}
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )

endfunction(add_all_executables)


file(GLOB gurls_headers RELATIVE ${PROJECT_SOURCE_DIR} ${HDRDIR}*.h*)
file(GLOB gurls_sources RELATIVE ${PROJECT_SOURCE_DIR} ${SRCDIR}*.cpp)


set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package( Boost COMPONENTS serialization date_time  filesystem unit_test_framework system REQUIRED)
option(Boost_USE_STATIC_LIBS "Link statically against boost libs" OFF)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

include_directories( ${HDRDIR} )


set(USE_BINARY_ARCHIVES OFF CACHE BOOL "If ON all the data structures in GURLS are saved/loaded using binary files.")

if(USE_BINARY_ARCHIVES)
    add_definitions(-DUSE_BINARY_ARCHIVES)
endif(USE_BINARY_ARCHIVES)

set(BUILD_EXPERIMENTAL OFF CACHE BOOL "If ON builds the experimental sources.")

if(BUILD_EXPERIMENTAL)
    add_definitions(-DEXPERIMENTAL)
endif(BUILD_EXPERIMENTAL)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build a shared library (ON) or a static one (OFF)")
if(BUILD_SHARED_LIBS)
    set( LIB_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIB_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


#add_definitions( -Wall )


add_library(${GURLSLIBRARY} ${LIB_TYPE} ${gurls_headers} ${gurls_sources} )
target_link_libraries(${GURLSLIBRARY} ${Boost_LIBRARIES} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

INSTALL(TARGETS ${GURLSLIBRARY}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

if(MSVC)

    add_definitions(-D_SCL_SECURE_NO_WARNINGS)
    set_target_properties(${GURLSLIBRARY} PROPERTIES COMPILE_FLAGS "/EHa")

    if(BUILD_SHARED_LIBS)
        set_target_properties(${GURLSLIBRARY} PROPERTIES COMPILE_DEFINITIONS "_GURLS_EXPORTS")
    endif(BUILD_SHARED_LIBS)

endif(MSVC)

set(GURLS_LINK_LIBRARIES
    ${GURLSLIBRARY}
    ${Boost_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${BLAS_LIBRARIES}
)

set(BUILD_DEMO OFF CACHE BOOL "")
if(BUILD_DEMO)
    add_all_executables(${PROJECT_SOURCE_DIR} ${DEMODIR} ${GURLS_LINK_LIBRARIES})
endif(BUILD_DEMO)

set(BUILD_TEST OFF CACHE BOOL "")
if(BUILD_TEST)
    set(GURLS_DATA_DIR "" CACHE PATH "Path to the Gurls++ data directory")

    if(GURLS_DATA_DIR STREQUAL "")
        message(WARNING, "GURLS_DATA_DIR value is set to an empty string. Please be sure this is the right configuration for you")
    endif(GURLS_DATA_DIR STREQUAL "")

    add_definitions(-DGURLS_DATA_DIR="${GURLS_DATA_DIR}")

    add_all_executables(${PROJECT_SOURCE_DIR} ${TESTDIR} ${GURLS_LINK_LIBRARIES})
endif(BUILD_TEST)

set(BUILD_MISC OFF CACHE BOOL "")
if(BUILD_MISC)
    add_all_executables(${PROJECT_SOURCE_DIR} ${MISCDIR} ${GURLS_LINK_LIBRARIES})
endif(BUILD_MISC)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
		)
endif(DOXYGEN_FOUND)
