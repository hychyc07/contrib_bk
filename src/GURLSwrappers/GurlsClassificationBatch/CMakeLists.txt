# Copyright: (C) 2013 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu  elena.ceseracciu@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME GurlsClassificationBatch)
project(${PROJECTNAME})

include_directories(${YARP_INCLUDE_DIRS} ${Gurls++_INCLUDE_DIRS} ${GurlsClassificationInterface_INCLUDE_DIRS})
set(hdrs GurlsClassificationBatchModule.h TrainingDatabase.h)
set(srcs main.cpp GurlsClassificationBatchModule.cpp TrainingDatabase.cpp)
add_executable(${PROJECTNAME} ${srcs} ${hdrs})
target_link_libraries(${PROJECTNAME} gurls++ ${YARP_LIBRARIES} GurlsClassificationInterface)