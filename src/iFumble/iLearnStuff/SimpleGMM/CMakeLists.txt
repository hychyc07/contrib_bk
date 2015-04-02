cmake_minimum_required(VERSION 2.8)
PROJECT(SimpleGMM)
SET(LIBRARYNAME simplegmm)
SET(TESTNAME testgmr)
SET(MODULENAME SimpleGMRModule)
SET(TESTMODULENAME TestGMRMod)
#link_directories ("/opt/icub/svn20100715/robotcub/trunk/iCub/src/lasaImitation/MathLib/lib")

#PROJECT(${TESTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source src/SimpleGMR.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${actionPrimitives_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
		    include
)
#"/opt/icub/svn20100715/robotcub/trunk/iCub/src/lasaImitation/MathLib/include/")

#ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})
#add_library(${PROJECTNAME} SHARED ${folder_source})
add_library(${LIBRARYNAME} STATIC src/SimpleGMR.cpp src/LasaMatrix.cpp src/LasaVector.cpp)
ADD_EXECUTABLE(${TESTNAME} src/testgmr.cpp)
ADD_EXECUTABLE(${MODULENAME} src/SimpleGMRModule.cpp)
ADD_EXECUTABLE(${TESTMODULENAME} src/TestGMRModule.cpp)

TARGET_LINK_LIBRARIES(${LIBRARYNAME} #icubmod
                                     #actionPrimitives
				      #MathLib
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
)

TARGET_LINK_LIBRARIES(${TESTNAME} #icubmod
                                     #actionPrimitives
				      ${LIBRARYNAME} 
				     # MathLib
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
)


TARGET_LINK_LIBRARIES(${MODULENAME} #icubmod
                                     #actionPrimitives
				      ${LIBRARYNAME} 
				    #  MathLib
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
)



TARGET_LINK_LIBRARIES(${TESTMODULENAME} 
				    #  MathLib
				    ${LIBRARYNAME} 
                                     ${YARP_LIBRARIES}
)

INSTALL(TARGETS ${MODULENAME} DESTINATION $ENV{ICUB_ROOT}/contrib/src/iFumble/bin)   
INSTALL(TARGETS ${TESTMODULENAME} DESTINATION $ENV{ICUB_ROOT}/contrib/src/iFumble/bin)   




