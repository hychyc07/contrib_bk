##########
Installing
##########
Installing ARToolKit

install libxmu
libv4l-dev
libv4l-dev libxi-dev libxmu-dev
or for gstreamer: libgstreamer0.10-dev
More detailes here: http://www.brunosilva.info/2011/07/installing-artoolkit-on-ubuntu.html
For Ubuntu 11.04 and above: 
changed the 'linux/videodev.h' header name for 'libv4l1-videodev.h'
in files: include/AR/sys/videoLinuxV4L.h
lib/SRC/VideoLinuxV4L/video.c

Download latest version from
http://www.hitl.washington.edu/artoolkit/
Extract the tar file and switch to the directory
Configure
____________
./Configure
____________

And enter the informations according to your system.
Make it
___________
make
___________


Installing ARTrackerUH

change CMakeList.txt

INCLUDE_DIRECTORIES(<PATH>/ARToolKit/include/AR/ ${OPENCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
LINK_LIBRARIES(<PATH>/ARToolKit/lib/libAR.a ${OPENCV_LIBRARIES} ${ICUB_LIBRARIES} ${YARP_LIBRARIES})
 

change src/ARToolKitTracker.cpp

#include <<PATH>/ARToolKit/include/AR/ar.h>
#include <<PATH>/ARToolKit/include/AR/param.h>

change include/iCub/ARToolKitTracker.h

// ARToolkit
#include <<PATH>/ARToolKit/include/AR/ar.h>
#include <<PATH>/ARToolKit/include/AR/param.h>

change include/iCub/arMarkerDetectorModule.h

// ARToolkit

#include <<PATH>/ARToolKit/include/AR/ar.h>

#####
USAGE
#####
Start the ARTrackerUH


##############
Change the iCubARMarkerDetectorModule.ini (paths)
_____________________
Go to the location where you have installed the ARTrackerUH and run the binary
start a yarpserver
 
./yarpdev --device opencv_grabber --name /cam

./ARTrackerUH --file <path>/iCubARMarkerDetectorModule.ini

./yarp connect /cam /ARTrackerUH/img:i 

You can find the iCubARMarkerDetectorModule.ini file in the location in which you checked out the ARTrackerUH.
