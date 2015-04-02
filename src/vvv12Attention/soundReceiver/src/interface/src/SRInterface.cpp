/*
 * SRInterface.cpp
 *
 *  Created on: Jul 19, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SRInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;
using namespace std;

bool SRInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name         : changes the rootname of the module ports \n");
		printf(
				"--robot        : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--soundInPort : changes the name for the sound input port to /rootname/<new name> \n");
		printf(
				"--mapOutPort : changes the name for the saliancy map output port to /rootname/<new name> \n");
		printf(
				"--toFile : Writes the sound input to the given destination <path>/<name>.wav.\n");
		printf("--threshold : Threshold for volume comparance.\n");
		printf("--highPassFreq : High pass filter cutoff frequency.\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("soundReceiver"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	soundInPortName = "/";
	soundInPortName += getName(
			rf.check("soundInPort", Value("/sound:i"),
					"Input sound port (string)").asString());

	mapOutPortName = "/";
	mapOutPortName += getName(
			rf.check("mapOutPortPort", Value("/map:o"),
					"Saliancy map output port (string)").asString());

	/* opening ports */
	if (!soundInPort.open(soundInPortName.c_str())) {
		cout << getName() << ": unable to open port " << soundInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	if (!mapOutPort.open(mapOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << mapOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Getting parameters */
	filename =
			rf.check("toFile", Value(""), "path to new file (string)").asString();
	threshold = rf.check("threshold", Value(1000),
			"threshold for volume comparance (int)").asInt();
	highPassFreq = rf.check("highPassFreq", Value(20000),
			"high pass filter cutoff frequency (int)").asInt();
	highPassFreq *= 2;

	/* Starting threads */
	cout << "Running threads:" << endl;
	soundPortReader = new SoundPortReader(&soundInPort, &mapOutPort, filename, threshold,
			highPassFreq);
	soundPortReader->start();

	return true;
}

bool SRInterface::interruptModule() {
	soundInPort.interrupt();
	mapOutPort.interrupt();
	return true;
}

bool SRInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	soundPortReader->stopLoop();
	soundPortReader->stop();
	cout << " Closing ports" << endl;
	soundInPort.close();
	mapOutPort.close();
	return true;
}

bool SRInterface::respond() {
	return true;
}

double SRInterface::getPeriod() {
	return 0.1;
}

bool SRInterface::updateModule() {
	return true;
}

