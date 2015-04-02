/*
 * SRInterface.cpp
 *
 *  Created on: Jun 27, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SRInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool SRInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name : changes the rootname of the module ports \n");
		printf(
				"--robot : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--resultsPort : changes the name of the results port to /rootname/<new name>\n");
		printf(
				"--soundPort : changes the name of the sound port to /rootname/<new name>\n");
		printf(
				"--triggerPort : changes the name of the recording trigger port to /rootname/<new name>\n");
		printf(
				"--workdir : Specify the directory where the audio files should be stored.\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("SoundRecorder"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	resultsPortName = "/";
	resultsPortName += getName(
			rf.check("resultsPort", Value("/Results:o"),
					"Output result data port (string)").asString());

	soundPortName = "/";
	soundPortName += getName(
			rf.check("soundPort", Value("/Sound:o"),
					"Output sound port (string)").asString());

	triggerPortName = "/";
	triggerPortName += getName(
			rf.check("triggerPort", Value("/Trigger:i"),
					"Trigger recording port (string)").asString());

	/* Getting file names */
	workdir = rf.check("workdir", Value(""), "path (string)").asString();
	if (workdir == "") {
		cout << getName()
				<< ": Please specify a directory where the audio files should be stored."
				<< endl;
		return false;
	}

	/* Getting Parameters */
	amplify = rf.check("amplify", Value(10), "Amplify sound by this value (int)").asInt();

	/* Opening ports */
	if (!resultsPort.open(resultsPortName.c_str())) {
		cout << getName() << ": unable to open port " << resultsPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!soundPort.open(soundPortName.c_str())) {
		cout << getName() << ": unable to open port " << soundPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!triggerPort.open(triggerPortName.c_str())) {
		cout << getName() << ": unable to open port " << triggerPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Starting threads */
	cout << "Running threads:" << endl;
	soundGrabber = new SoundGrabber(&soundPort, &resultsPort, workdir, amplify);
	soundGrabber->start();
	controleThread = new ControleThread(&triggerPort, soundGrabber);
	controleThread->start();
	return true;
}

bool SRInterface::interruptModule() {
	resultsPort.interrupt();
	soundPort.interrupt();
	triggerPort.interrupt();
	return true;
}

bool SRInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	controleThread->stopLoop();
	controleThread->stop();
	soundGrabber->stopLoop();
	soundGrabber->stop();
	cout << " Closing ports" << endl;
	resultsPort.close();
	soundPort.close();
	triggerPort.close();

	delete soundGrabber;

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

