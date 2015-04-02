/*
 * main.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

/**
 * @mainpage LoomingDetector
 *
 * This yarp module is used to detect looming behavior via the Microsoft
 * Kinect. In order to use this module you will have to run the yarp KinectDeviceLocal
 * on the machine with the Kinect. In future releases there may be an inclusion
 * of the IKinectDevice but at the moment you have have to run the yarp device
 * yourself.@n
 * It is supposed to work with the Kinect information and an object tracker of any kind.
 * The object tracker feature is not yet implemented.
 *
 * @section Installation
 * @subsection Dependencies
 * There are no other dependencies than the usual iCub dependencies.
 * @subsection Building
 * -# Create a @c build directory in the loomingDetector root directory.
 *  -# Change to that directory
 * -# Run @c cmake @c ..
 * -# @c make
 * -# @c make install
 *
 * @section Usage
 * -# Start the KinectDeviceLocal on the machine with access to the Kinect:
 * @n @c yarpdev @c --device KinectDeviceLocal @c --portPrefix @c /kinect @c --userDetection
 * -# Start the LoomingDetector. You do not have to give any parameters in order for
 * it to work. Try --help if you want to change anything.
 * -# Connect:
 * @n /kinect/skeleton:o to /LoomingDetector/skeleton:i
 * -# There are two output ports:
 *  - /LoomingDetector/looming:o @n This port sends a start and end marker for looming
 *  actions and streams the position of the hand during the looming phase.
 *  - /LoomingDetector/data:o @n
 *  This port streams mostly debugging and raw data.
 *
 *  @author Christian Dondrup
 *
 */

#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include "iCub/LDInterface.h"


using namespace yarp::os;
using namespace std;


int main(int argc, char * argv[]){

	Network yarp;

	LDInterface module;

	ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("");      //overridden by --from parameter
    rf.setDefaultContext("");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

	module.runModule(rf);
	return 0;
}




