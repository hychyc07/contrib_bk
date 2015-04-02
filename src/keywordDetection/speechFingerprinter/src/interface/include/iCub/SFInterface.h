/*
 * SFInterface.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef SFINTERFACE_H_
#define SFINTERFACE_H_

#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/Fingerprinter.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**
 * This class represents the main thread of the program.
 * It starts all sub modules and takes control of their runtime behavior.
 */
class SFInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string inputPortName;
	string outputPortName;
	string audiofile;
	string workdir;
	string filters;
	bool overwrite;

	Port inPort, outPort;


	/* class variables */

	Fingerprinter* fingerprinter;


public:
	/**
	 * @brief configure all the module parameters and return true if successful
	 * @param rf The yarp::os::ResourceFinder used by this program
	 */
	bool configure(yarp::os::ResourceFinder &rf);

	/**
	 * @brief Interrupt, e.g., the ports
	 */
	bool interruptModule();

	/**
	 * @brief Close and shut down the module
	 */
	bool close();
	bool respond();
	double getPeriod();
	bool updateModule();
};

#endif /* SFINTERFACE_H_ */
