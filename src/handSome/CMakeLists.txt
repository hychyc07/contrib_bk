CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME HandSome)

FILE(GLOB folder_header include/*.hpp)
#FILE(GLOB folder_source src/*.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
#SOURCE_GROUP("Source Files" FILES ${folder_source})


PROJECT(${PROJECTNAME})	

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
					
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

#Add each "library class"
ADD_LIBRARY(handsome_lib src/controller.cpp  src/impController.cpp src/gazeController.cpp src/robot.cpp src/CartesianImpedance.cpp src/JointImpedance.cpp ${folder_header})

#The following two lines for each executale
ADD_EXECUTABLE(testGaze apps/TestGaze.cpp )
ADD_EXECUTABLE(testMain apps/TestMain.cpp )
ADD_EXECUTABLE(Demo apps/DemoControl.cpp )
ADD_EXECUTABLE(testRobot apps/testRobot.cpp )
ADD_EXECUTABLE(testCartesianImpedance apps/TestCartesianImpedance.cpp )
ADD_EXECUTABLE(testAutomaticUpdate apps/testAutomaticUpdate.cpp )

TARGET_LINK_LIBRARIES(testGaze handsome_lib ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(testMain handsome_lib ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(Demo handsome_lib iDyn ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(testRobot handsome_lib iDyn ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(testCartesianImpedance handsome_lib iDyn ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(testAutomaticUpdate handsome_lib iDyn ${YARP_LIBRARIES})
