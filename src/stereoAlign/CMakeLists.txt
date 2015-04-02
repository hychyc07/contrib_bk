CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME stereoAlign)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

    SET(USE_SIFT_GPU ${IS_SIFT_GPU} CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")

    if (USE_SIFT_GPU)
        FIND_PACKAGE(SIFTGPU)
        if(SIFTGPU_FOUND)
            include_directories( ${SIFTGPU_INCLUDE_DIRS})
        endif()
    endif()

    FIND_PACKAGE(YARP)
    FIND_PACKAGE(ICUB)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
    INCLUDE(iCubOptions)
    INCLUDE(iCubHelpers)

    FIND_PACKAGE(OpenCV)

    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
    SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

    IF (${OpenCV_VERSION} VERSION_LESS 2.4.0 AND NOT USE_SIFT_GPU )
        MESSAGE("You seem to have OPENCV < 2.4, SIFT extraction will be very slow, please update OpenCV or consider using SIFT GPU")
    ELSEIF (USE_SIFT_GPU AND SIFTGPU_FOUND )
        MESSAGE("OK now setting up for using SIFT GPU")
    ELSE ()
        MESSAGE("You seem to have OPENCV > 2.4, This will run ok, but you might also consider using SIFT GPU")
    ENDIF()

    # Search for source code.
    if (USE_SIFT_GPU)
        SET(folder_source src/main.cpp src/stereoAlignGPU.cpp)
    ELSE()
        SET(folder_source src/main.cpp src/stereoAlign.cpp)
    ENDIF()
    SET(folder_header include/iCub/stereoAlign.h)

    if (USE_SIFT_GPU)
        ADD_DEFINITIONS(-DUSING_GPU)
    ENDIF()

    source_group("Source Files" FILES ${folder_source})
    source_group("Header Files" FILES ${folder_header})

    INCLUDE_DIRECTORIES(
                        ${PROJECT_SOURCE_DIR}/include
                        ${YARP_INCLUDE_DIRS}
                        ${ICUB_INCLUDE_DIRS}
                        ${OpenCV_INCLUDE_DIRS}
		                )

    ADD_EXECUTABLE( ${PROJECTNAME} ${folder_source} ${folder_header}) 			

    if (USE_SIFT_GPU)
        TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${SIFTGPU_LIBRARIES} )
    else ()
        TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} )
    endif()

    INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)
    ADD_SUBDIRECTORY(app)
    

