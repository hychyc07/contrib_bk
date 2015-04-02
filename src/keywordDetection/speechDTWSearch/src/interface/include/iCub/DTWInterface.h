/*
 * SFInterface.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef DTWINTERFACE_H_
#define DTWINTERFACE_H_

#include <iostream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/WarpingThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**
 * This class represents the main thread of the program.
 * It starts all sub modules and takes control of their runtime behavior.
 */
class DTWInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string inputPortName;
	string outputPortName;
	string filelist;
	string workdir;
	string filters;
	string outfile;
	bool overwrite;

	Port inPort, outPort;


	/* class variables */
	WarpingThread *warpingThread;


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

#endif /* DTWINTERFACE_H_ */
