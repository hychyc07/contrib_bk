/*
 * LDInterface.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/LDInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;
using namespace std;

bool LDInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name         : changes the rootname of the module ports \n");
		printf(
				"--robot        : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--skeletonPort : changes the name for the skeleton input port to /rootname/<new name> \n");
		printf(
				"--objectPort : changes the name for the object input port to /rootname/<new name> \n");
		printf(
				"--outputPort : changes the name for the output port to /rootname/<new name>. "
						"This port send the beginning and end of the looming action of each hand.\n");
		printf(
				"--dataOutPort : changes the name for the data output port to /rootname/<new name>. "
						"This port sends the calculated data like mean, variance and distance. \n");
		printf(
				"--habituationThreshold : Setting the habituation threshold to the given int value."
						" After the given number of looming detections in a row the mean will be reset to the new position.\n");
		printf(
				"--emulator :  starting the loomingDetector in emulator mode without an actual kinect.\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the emulator value */
	double emulator = 0.0;

	emulator =
			rf.check("emulator", Value(0.0),
					"starting the loomingDetector in emulator mode without an actual kinect "
							"specifying the distance at which looming will be detected.").asDouble();
	if (emulator) {
		cout << "Starting LoomingDetector in emulator mode." << endl
				<< "Using a looming distance of: " << emulator << endl;
	}

	/* Getting Parameters */

	habituationThreshold = rf.check("habituationThreshold", Value(2000),
			"Habituation threshold (int)").asInt();

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("LoomingDetector"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	skeletonPortName = "/";
	skeletonPortName += getName(
			rf.check("skeletonPort", Value("/skeleton:i"),
					"Input skeleton data port (string)").asString());

	objectPortName = "/";
	objectPortName += getName(
			rf.check("objectPort", Value("/object:i"),
					"Input object data port (string)").asString());

	outputPortName = "/";
	outputPortName += getName(
			rf.check("outputPort", Value("/looming:o"),
					"Output looming data port (string)").asString());

	dataOutPortName = "/";
	dataOutPortName += getName(
			rf.check("dataOutPort", Value("/data:o"),
					"Output data port (string)").asString());

	/* opening ports */
	if (!skeletonPort.open(skeletonPortName.c_str())) {
		cout << getName() << ": unable to open port " << skeletonPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!objectPort.open(objectPortName.c_str())) {
		cout << getName() << ": unable to open port " << objectPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!outPort.open(outputPortName.c_str())) {
		cout << getName() << ": unable to open port " << outputPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!dataPort.open(dataOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << dataOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Starting threads */
	cout << "Running threads:" << endl;
	loomingDetector = new LoomingDetector(
			new LoomingWriter(&outPort, &dataPort), emulator, habituationThreshold);
	loomingDetector->start();
	dataCollector = new DataCollector(loomingDetector);
	skeletonThread = new SkeletonReader(&skeletonPort, dataCollector,
			emulator ? true : false);
	objectThread = new ObjectReader(&objectPort, dataCollector);
	skeletonThread->start();
	objectThread->start();

	return true;
}

bool LDInterface::interruptModule() {
	skeletonPort.interrupt();
	objectPort.interrupt();
	outPort.interrupt();
	dataPort.interrupt();
	return true;
}

bool LDInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Closing ports" << endl;
	skeletonPort.close();
	objectPort.close();
	outPort.close();
	dataPort.close();
	cout << " Stopping threads" << endl;
	//Calling the stopLoop() functions to end the endless loop.
	skeletonThread->stopLoop();
	skeletonThread->stop();
	objectThread->stopLoop();
	objectThread->stop();
	loomingDetector->stopLoop();
	loomingDetector->stop();
	return true;
}

bool LDInterface::respond() {
	return true;
}

double LDInterface::getPeriod() {
	return 0.1;
}

bool LDInterface::updateModule() {
	return true;
}

