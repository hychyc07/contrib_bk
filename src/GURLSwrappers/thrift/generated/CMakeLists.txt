set(PROJECTNAME GurlsClassificationInterface)
project(${PROJECTNAME})

include(GurlsClassification_thrift.cmake)

include_directories(include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)

export(TARGETS ${PROJECTNAME} FILE ${PROJECTNAME}.cmake)

install(TARGETS ${PROJECTNAME} EXPORT ${PROJECTNAME} DESTINATION lib)
install(EXPORT ${PROJECTNAME} DESTINATION lib/${PROJECTNAME})

export(PACKAGE ${PROJECTNAME})


get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
set(${PROJECTNAME}_INCLUDE_DIRS ${inc_dirs} CACHE INTERNAL "Generated-files libraries include directory")
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECTNAME}Config.cmake "set(\${PROJECTNAME}_INCLUDE_DIRS ${PROJECTNAME}_INCLUDE_DIRS)\ninclude(\${PROJECTNAME})\n" )

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECTNAME}ConfigForInstall.cmake "set(\${PROJECTNAME}_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)\ninclude(\${PROJECTNAME})\n" )
install(FILES ${PROJECT_BINARY_DIR}/${PROJECTNAME}ConfigForInstall.cmake RENAME ${PROJECTNAME}Config.cmake DESTINATION lib/${PROJECTNAME})
