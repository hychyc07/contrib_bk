/*
 * SRInterface.cpp
 *
 *  Created on: Jul 19, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SEInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;
using namespace std;

bool SEInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name         : changes the rootname of the module ports \n");
		printf(
				"--robot        : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--soundOutPort : changes the name for the sound input port to /rootname/<new name> \n");
		printf(
				"--fromFile : Reads the sound from the given destination <path>/<name>.wav.\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("soundEmulator"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	soundOutPortName = "/";
	soundOutPortName += getName(
			rf.check("soundOutPort", Value("/sound:o"),
					"Input sound port (string)").asString());

	/* opening ports */
	if (!soundOutPort.open(soundOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << soundOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Getting parameters */
	filename =
			rf.check("fromFile", Value(""), "path to new file (string)").asString();

	/* Starting threads */
	cout << "Running threads:" << endl;
	chunkSender = new ChunkSender(&soundOutPort, filename);
	chunkSender->start();

	return true;
}

bool SEInterface::interruptModule() {
	soundOutPort.interrupt();
	return true;
}

bool SEInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	chunkSender->stopLoop();
	chunkSender->stop();
	cout << " Closing ports" << endl;
	soundOutPort.close();
	return true;
}

bool SEInterface::respond() {
	return true;
}

double SEInterface::getPeriod() {
	return 0.0;
}

bool SEInterface::updateModule() {
	return true;
}

