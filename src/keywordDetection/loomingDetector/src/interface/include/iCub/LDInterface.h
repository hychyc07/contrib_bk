/*
 * LDInterface.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef LDINTERFACE_H_
#define LDINTERFACE_H_

#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/SkeletonReader.h"
#include "iCub/ObjectReader.h"
#include "iCub/LoomingDetector.h"
#include "iCub/DataCollector.h"
#include "iCub/LoomingWriter.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;

/**
 * This is the main thread of the program which starts and manages all other threads.
 */
class LDInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string skeletonPortName;
	string objectPortName;
	string outputPortName;
	string dataOutPortName;
	int habituationThreshold;

	/* class variables */

	BufferedPort<Bottle> skeletonPort;
	BufferedPort<Bottle> objectPort;
	Port outPort;
	BufferedPort<Bottle> dataPort;
	SkeletonReader* skeletonThread;
	ObjectReader* objectThread;
	LoomingDetector* loomingDetector;
	DataCollector* dataCollector;

public:
	/**
	 * @brief configure all the module parameters and return true if successful
	 * @param rf The yarp::os::ResourceFinder used by this program
	 */
	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful

	/**
	 * @brief Interrupt, e.g., the ports
	 */
	bool interruptModule(); // interrupt, e.g., the ports

	/**
	 * @brief Close and shut down the module
	 */
	bool close(); // close and shut down the module
	bool respond();
	double getPeriod();
	bool updateModule();
};

#endif /* LDINTERFACE_H_ */
