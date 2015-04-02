set(PROJECTNAME thriftGeneratedDMP)
project(${PROJECTNAME})

set(THRIFT_HEADERS)
set(THRIFT_SOURCES)
#file(GLOB fileList "*.cmake")
set(fileList iCub_dmpManager_thrift.cmake iCub_dmpLearner_thrift.cmake iCub_dmpExecutor_thrift.cmake)
foreach(scriptFile ${fileList})
  include(${scriptFile})
  list(APPEND THRIFT_HEADERS ${headers})
  list(APPEND THRIFT_SOURCES ${sources})
endforeach()
message ("Thrift headers ${THRIFT_HEADERS}.\n Thirft sources: ${THRIFT_SOURCES}")
include_directories(include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${THRIFT_HEADERS} ${THRIFT_SOURCES})
set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)

export(TARGETS ${PROJECTNAME} FILE ${PROJECTNAME}.cmake)

install(TARGETS ${PROJECTNAME} EXPORT ${PROJECTNAME} DESTINATION lib)
install(EXPORT ${PROJECTNAME} DESTINATION lib/${PROJECTNAME})

export(PACKAGE ${PROJECTNAME})
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
configure_file(libraryConfig.cmake.in ${PROJECT_BINARY_DIR}/${PROJECTNAME}Config.cmake @ONLY)
get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
set(${PROJECTNAME}_INCLUDE_DIRS ${inc_dirs} CACHE INTERNAL "Generated-files libraries include directory")
#icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include  #should add ${YARP_INCLUDE_DIRS} as EXTERNAL_INCLUDE_DIRS ...?
#                                   DESTINATION ./ # THRIFT_HEADERS already have the include/iCub/etc path
#                                   FILES_WITH_PATH ${THRIFT_HEADERS} VERBOSE)
#set(generated_libs_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Generated-files libraries include directory")
