include(YarpIDL)
set(generated_files_dir "${CMAKE_CURRENT_SOURCE_DIR}/generated")
yarp_idl_to_dir("GurlsClassification.thrift" ${generated_files_dir})

add_subdirectory(generated)


