/*
 * SFInterface.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SFInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::dev;
using namespace std;

bool SFInterface::configure(yarp::os::ResourceFinder &rf) {

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
				"--audio : Specify the audio file or a filelist.txt which should be fingerprinted. \n");
		printf(
				"--workdir : Specify the directory where the fingerprint files should be stored."
						"These files will be given unique file names containing the system time. \n");
		printf("--filter : The filter descriptor file. \n");
		printf("--overwrite: Overwrites existing keys and db files\n");
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
	audiofile = rf.check("audio", Value(""), "file name (string)").asString();
	if (audiofile == "") {
		cout << getName()
				<< ": No audio file specified. Running in online mode expecting input through port."
				<< endl;
	}

	workdir = rf.check("workdir", Value(""), "path (string)").asString();
	if (workdir == "") {
		cout << getName()
				<< ": Please specify a directory where the fingerprints should be stored."
				<< endl;
		return false;
	}

	filters = rf.check("filter", Value(""), "file name (string)").asString();
	if (filters == "") {
		cout << getName() << ": Please specify a filter descriptor file"
				<< endl;
		return false;
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
	fingerprinter = new Fingerprinter(audiofile, filters, workdir, &inPort, &outPort,
			overwrite);
	if (!fingerprinter->start())
		return false;

	return true;
}

bool SFInterface::interruptModule() {
	inPort.interrupt();
	return true;
}

bool SFInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	fingerprinter->stopLoop();
	fingerprinter->stop();
	cout << " Closing ports" << endl;
	inPort.close();
	//No ports to close
	return true;
}

bool SFInterface::respond() {
	return true;
}

double SFInterface::getPeriod() {
	return 0.1;
}

bool SFInterface::updateModule() {
	return fingerprinter->isRunning();
}

