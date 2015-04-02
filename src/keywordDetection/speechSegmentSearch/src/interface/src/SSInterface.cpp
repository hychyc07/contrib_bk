/*
 * SSInterface.cpp
 *
 *  Created on: Jun 27, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SSInterface.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool SSInterface::configure(yarp::os::ResourceFinder &rf) {

	/* Help message */
	if (rf.check("help")) {
		printf("HELP \n");
		printf("--name : changes the rootname of the module ports \n");
		printf(
				"--robot : changes the name of the robot which the module interfaces to  \n");
		printf(
				"--outputPort : changes the name of the output port to /rootname/<new name>\n");
		printf(
				"--inputPort : changes the name of the input port to /rootname/<new name>\n");
		printf(
				"--audio : Specify the audio file or a filelist.txt which should be searched "
						"for in the database. \n");
		printf(
				"--workdir : Specify the directory where the fingerprint files are stored."
						"These files will be search for matches with the given audio file(s). \n");
		printf("--em : Specify which em parameter file should be used.\n");
		printf("--filter : The filter descriptor file. \n");
		printf("--outfile : A file to save the results to. \n");
		printf(
				"--nbest : Sets how many of the best results should be in the output.\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("SpeechSegmentSearch"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	outputPortName = "/";
	outputPortName += getName(
			rf.check("outputPort", Value("/SearchResults:o"),
					"Output data port (string)").asString());

	inputPortName = "/";
	inputPortName += getName(
			rf.check("inputPort", Value("/FileNames:i"),
					"Input data port (string)").asString());

	/* Getting file names */
	audiofile = rf.check("audio", Value(""), "file name (string)").asString();
	if (audiofile == "") {
		cout << getName()
				<< ": No audio file specified. Running in online mode." << endl;
	}

	workdir = rf.check("workdir", Value(""), "path (string)").asString();
	if (workdir == "") {
		cout << getName()
				<< ": Please specify a directory where the fingerprints should be stored."
				<< endl;
		return false;
	}

	filterfile = rf.check("filter", Value(""), "file name (string)").asString();
	if (filterfile == "") {
		cout << getName() << ": Please specify a filter descriptor file"
				<< endl;
		return false;
	}

	emfile = rf.check("em", Value(""), "file name (string)").asString();
	if (emfile == "") {
		cout << getName() << ": Please specify an em parameter file" << endl;
		return false;
	}

	outfile =
			rf.check("outfile", Value(""), "out file name (string)").asString();
	if (emfile == "") {
		cout << getName()
				<< ": No output file specified. Results will not be saved."
				<< endl;
	}

	/* Getting parameters */

	nbest =
			rf.check("nbest", Value(5), "Number of results per file (int)").asInt();

	/* Opening ports */
	if (!outPort.open(outputPortName.c_str())) {
		cout << getName() << ": unable to open port " << outputPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}
	if (!inPort.open(inputPortName.c_str())) {
		cout << getName() << ": unable to open port " << inputPortName << endl;
		return false; // unable to open; let RFModule know so that it won't run
	}

	/* Starting threads */
	cout << "Running threads:" << endl;
	matchWriter = new MatchWriter(&outPort);
	segserach = new SegmentSearch(audiofile, filterfile, workdir, emfile,
			matchWriter, &inPort, nbest, outfile);
	if (!segserach->start())
		return false;
	return true;
}

bool SSInterface::interruptModule() {
	outPort.interrupt();
	inPort.interrupt();
	return true;
}

bool SSInterface::close() {
	cout << "Closing module: " << getName() << ":" << endl;
	cout << " Stopping threads" << endl;
	segserach->stopLoop();
	segserach->stop();
	cout << " Closing ports" << endl;
	outPort.close();
	inPort.close();
	return true;
}

bool SSInterface::respond() {
	return true;
}

double SSInterface::getPeriod() {
	return 0.1;
}

bool SSInterface::updateModule() {
	return segserach->isRunning();
}

