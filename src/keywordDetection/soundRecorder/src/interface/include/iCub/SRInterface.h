/*
 * SRInterface.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef SRINTERFACE_H_
#define SRINTERFACE_H_

#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include "iCub/SoundGrabber.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**
 * This class represents the main thread of the program.
 * It starts all sub modules and takes control of their runtime behavior.
 */
class SRInterface: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string robotPortName;
	string resultsPortName;
	string soundPortName;
	string triggerPortName;
	string workdir;
	int amplify;



	/* class variables */
	BufferedPort<Sound> soundPort;
	Port triggerPort, resultsPort;

	SoundGrabber* soundGrabber;
	ControleThread* controleThread;


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

#endif /* SRINTERFACE_H_ */
