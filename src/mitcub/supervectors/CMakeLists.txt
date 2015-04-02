if(USE_SUPERVECTORS)
if(SIFTGPU_FOUND)

   find_package(YARP)
   find_package(ICUB)
   set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
   set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
   include(iCubHelpers)


   include_directories(${SIFTGPU_INCLUDE_DIRS})
   add_executable(supervectors main.cpp)
   target_link_libraries(supervectors ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${SIFTGPU_LIBRARIES})
   install(TARGETS supervectors RUNTIME DESTINATION ${INSTALL_PATH})

endif()
endif()
