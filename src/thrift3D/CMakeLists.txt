cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME thrift3D)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

# add yarp definitions
add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT

# pick up yarp's cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

INCLUDE(${YARP_MODULE_PATH}/YarpIDL.cmake)
SET(generated_files_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(iCub/data3D/objects3D.thrift ${generated_files_dir})

set(THRIFT_HEADERS)
set(THRIFT_SOURCES)

set(fileList iCub_data3D_objects3D_thrift.cmake)
foreach(scriptFile ${fileList})
  include(${scriptFile})
  list(APPEND THRIFT_HEADERS ${headers})
  list(APPEND THRIFT_SOURCES ${sources})
endforeach()

include_directories(include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${THRIFT_HEADERS} ${THRIFT_SOURCES})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)

export(TARGETS ${PROJECTNAME} FILE ${PROJECTNAME}.cmake)

install(TARGETS ${PROJECTNAME} EXPORT ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/lib)
install(EXPORT ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/lib/${PROJECTNAME})

export(PACKAGE ${PROJECTNAME})
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
configure_file(libraryConfig.cmake.in ${PROJECT_BINARY_DIR}/${PROJECTNAME}Config.cmake @ONLY)
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
set(${PROJECTNAME}_INCLUDE_DIRS ${inc_dirs} CACHE INTERNAL "Generated-files libraries include directory")

