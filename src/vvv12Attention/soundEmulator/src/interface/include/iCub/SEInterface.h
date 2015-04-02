/*
 * SRInterface.h
 *
 *  Created on: Jul 19, 2012
 *      Author: Christian Dondrup
 */

#ifndef SEINTERFACE_H_
#define SEINTERFACE_H_

#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/ChunkSender.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;

/**
 * This is the main thread of the program which starts and manages all other threads.
 */
class SEInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string soundOutPortName;
	string filename;
	ChunkSender* chunkSender;
	/* class variables */


	BufferedPort<Sound> soundOutPort;

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

#endif /* SEINTERFACE_H_ */
