/*
 * SRInterface.cpp
 *
 *  Created on: Jun 27, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/AInterface.h"

using namespace yarp::os;
using namespace std;

bool AInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name : changes the rootname of the module ports \n");
		printf(
				"--robot : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--loomingInPort : changes the name of the looming data input port to /moduleName/<new name>\n");
		printf(
				"--loomingOutPort : changes the name of the looming data output port to /moduleName/<new name>\n");
		printf(
				"--soundRecInPort : changes the name of the sound recorder data input port to /moduleName/<new name>\n");
		printf(
				"--soundRecOutPort : changes the name of the sound recorder data output port to /moduleName/<new name>\n");
		printf(
				"--fingerprinterInPort : changes the name of the fingerprinter data input port to /moduleName/<new name>\n");
		printf(
				"--fingerprinterOutPort : changes the name of the fingerprinter data output port to /moduleName/<new name>\n");
		printf(
				"--segmenterInPort : changes the name of the segmenter data input port to /moduleName/<new name>\n");
		printf(
				"--segmenterOutPort : changes the name of the segmenter data output port to /moduleName/<new name>\n");
		printf(
				"--searchInPort : changes the name of the search data input port to /moduleName/<new name>\n");
		printf(
				"--searchOutPort : changes the name of the search data output port to /moduleName/<new name>\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName =
			rf.check("name", Value("Arbiter"), "module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	/* Input */
	loomingInPortName = "/";
	loomingInPortName += getName(
			rf.check("loomingInPort", Value("/looming:i"),
					"Looming data input port (string)").asString());
	soundRecInPortName = "/";
	soundRecInPortName += getName(
			rf.check("soundInPort", Value("/soundRec:i"),
					"Sound recorder data input port (string)").asString());
	fingerprinterInPortName = "/";
	fingerprinterInPortName += getName(
			rf.check("fingerprinterInPort", Value("/fingerprinter:i"),
					"Fingerprinter data input port (string)").asString());
	segmenterInPortName = "/";
	segmenterInPortName += getName(
			rf.check("segmenterInPort", Value("/segmenter:i"),
					"Segmenter data input port (string)").asString());
	searchInPortName = "/";
	searchInPortName += getName(
			rf.check("serachInPort", Value("/search:i"),
					"Search data input port (string)").asString());
	/* Output */
	loomingOutPortName = "/";
	loomingOutPortName += getName(
			rf.check("loomingOutPort", Value("/looming:o"),
					"Looming data input port (string)").asString());
	soundRecOutPortName = "/";
	soundRecOutPortName += getName(
			rf.check("soundOutPort", Value("/soundRec:o"),
					"Sound recorder data input port (string)").asString());
	fingerprinterOutPortName = "/";
	fingerprinterOutPortName += getName(
			rf.check("fingerprinterOutPort", Value("/fingerprinter:o"),
					"Fingerprinter data input port (string)").asString());
	segmenterOutPortName = "/";
	segmenterOutPortName += getName(
			rf.check("segmenterOutPort", Value("/segmenter:o"),
					"Segmenter data input port (string)").asString());
	searchOutPortName = "/";
	searchOutPortName += getName(
			rf.check("serachOutPort", Value("/search:o"),
					"Search data input port (string)").asString());

	/* Getting file names */

	/* Getting Parameters */

	/* Opening ports */
	if (!loomingInPort.open(loomingInPortName.c_str())) {
		cout << getName() << ": unable to open port " << loomingInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!loomingOutPort.open(loomingOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << loomingOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!soundRecInPort.open(soundRecInPortName.c_str())) {
		cout << getName() << ": unable to open port " << soundRecInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!soundRecOutPort.open(soundRecOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << soundRecOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!fingerprinterInPort.open(fingerprinterInPortName.c_str())) {
		cout << getName() << ": unable to open port " << fingerprinterInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!fingerprinterOutPort.open(fingerprinterOutPortName.c_str())) {
		cout << getName() << ": unable to open port "
				<< fingerprinterOutPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!segmenterInPort.open(segmenterInPortName.c_str())) {
		cout << getName() << ": unable to open port " << segmenterInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!segmenterOutPort.open(segmenterOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << segmenterOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!searchInPort.open(searchInPortName.c_str())) {
		cout << getName() << ": unable to open port " << searchInPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!searchOutPort.open(searchOutPortName.c_str())) {
		cout << getName() << ": unable to open port " << searchOutPortName
				<< endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Starting threads */
	cout << "Running threads:" << endl;
	readerManager = new ReaderManager(&loomingInPort, &searchInPort,
			&soundRecInPort, &segmenterInPort, &fingerprinterInPort);
	recorderControler = new RecorderControler(&soundRecOutPort);
	recorderControler->start();
	controlThread = new ControlThread(
			new WriterManager(&fingerprinterOutPort, &segmenterOutPort,
					&soundRecOutPort, &searchOutPort));
	controlThread->start();
	return true;
}

bool AInterface::interruptModule() {
	loomingInPort.interrupt();
	soundRecInPort.interrupt();
	fingerprinterInPort.interrupt();
	segmenterInPort.interrupt();
	searchInPort.interrupt();
	loomingOutPort.interrupt();
	soundRecOutPort.interrupt();
	fingerprinterOutPort.interrupt();
	segmenterOutPort.interrupt();
	searchOutPort.interrupt();
	return true;
}

bool AInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	delete readerManager;
	recorderControler->stopLoop();
	recorderControler->stop();
	controlThread->stopLoop();
	controlThread->stop();
	cout << " Closing ports" << endl;
	loomingInPort.close();
	soundRecInPort.close();
	fingerprinterInPort.close();
	segmenterInPort.close();
	searchInPort.close();
	loomingOutPort.close();
	soundRecOutPort.close();
	fingerprinterOutPort.close();
	segmenterOutPort.close();
	searchOutPort.close();
	return true;
}

bool AInterface::respond() {
	return true;
}

double AInterface::getPeriod() {
	return 0.1;
}

bool AInterface::updateModule() {
	return true;
}

