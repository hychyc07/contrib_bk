/*
 * SFInterface.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/DTWInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool DTWInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name : changes the rootname of the module ports \n");
		printf(
				"--robot : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--inPort : changes the name of the control input port to /moduleName/<new name> \n");
		printf(
				"--outPort : changes the name of the control output port to /moduleName/<new name> \n");
		printf(
				"--filelist : Specify the filelist.txt which should contains the audio files to be compared. \n");
		printf("--outfile : Specify a file to write the results to. \n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("SpeechFingerprinter"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting file names */
	filelist =
			rf.check("filelist", Value(""), "filelist name (string)").asString();
	if (filelist == "") {
		cout << getName()
				<< ": No audio filelist specified. Running in online mode expecting input through port."
				<< endl;
	}

	outfile =
			rf.check("outfile", Value(""), "out file name (string)").asString();
	if (filelist == "") {
		cout << getName() << ": No audio out file specified. Results will not be saved to a file." << endl;
	}

	/*Getting parameters*/
	overwrite = rf.check("overwrite", "overwrite old files (bool)");

	/*Getting port names*/
	inputPortName += "/";
	inputPortName +=
			getName(
					rf.check("inPort", Value("/control:i"),
							"port name (string)").asString());
	outputPortName += "/";
	outputPortName +=
			getName(
					rf.check("outPort", Value("/control:o"),
							"port name (string)").asString());

	/* Opening ports */
	if (!inPort.open(inputPortName.c_str())) {
		cout << getName() << ": unable to open port " << inputPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!outPort.open(outputPortName.c_str())) {
		cout << getName() << ": unable to open port " << outputPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Starting threads */
	cout << "Running threads:" << endl;
	warpingThread = new WarpingThread(filelist, &inPort, &outPort, outfile);
	warpingThread->start();

	return true;
}

bool DTWInterface::interruptModule() {
	inPort.interrupt();
	outPort.interrupt();
	return true;
}

bool DTWInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	warpingThread->stopLoop();
	warpingThread->stop();
	cout << " Closing ports" << endl;
	inPort.close();
	outPort.close();
	return true;
}

bool DTWInterface::respond() {
	return true;
}

double DTWInterface::getPeriod() {
	return 0.1;
}

bool DTWInterface::updateModule() {
	return warpingThread->isRunning();
}

