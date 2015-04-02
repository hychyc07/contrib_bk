/*
 * AInterface.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef AINTERFACE_H_
#define AINTERFACE_H_

#include <iostream>
#include <string>
#include <yarp/os/all.h>

#include "iCub/ReaderManager.h"
#include "iCub/WriterManager.h"
#include "iCub/RecorderControler.h"
#include "iCub/ControlThread.h"

using namespace std;
using namespace yarp::os;

/**
 * This class represents the main thread of the program.
 * It starts all sub modules and takes control of their runtime behavior.
 */
class AInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string loomingInPortName, soundRecInPortName, fingerprinterInPortName,
			segmenterInPortName, searchInPortName;
	string loomingOutPortName, soundRecOutPortName, fingerprinterOutPortName,
			segmenterOutPortName, searchOutPortName;
	Port loomingInPort, soundRecInPort, fingerprinterInPort, segmenterInPort,
			searchInPort;
	Port loomingOutPort, soundRecOutPort, fingerprinterOutPort,
			segmenterOutPort, searchOutPort;

	/* class variables */
	ReaderManager *readerManager;
	RecorderControler *recorderControler;
	ControlThread * controlThread;

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

#endif /* AINTERFACE_H_ */
