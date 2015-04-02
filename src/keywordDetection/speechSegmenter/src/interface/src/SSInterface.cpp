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
				"--workdir : Specify the directory where the segments should be stored.\n");
		printf(
				"--lowpass : The frequency at which the signal should be cut by the Low Pass filter. "
						"Low pass value has to be higher than high pass value.\n");
		printf(
				"--highpass : The frequency at which the signal should be cut by the High Pass filter. "
						"Low pass value has to be higher than high pass value.\n");
		printf("--prefac : The pre-emphasis filter factor.\n");
		printf("--mu : The bias for the threshold.\n");
		printf(
				"--lambda : The minimal phoneme length as an integer. lambda * window_size(23ms)\n");
		printf(
				"--delta : The maximal distance as an integer between two sound bursts to be counted as one. "
						"delta * window_length(23ms)\n");
		printf(
				"--min : If the stereo signal is not producing the number of segments for each channel then "
						"min will choose the result with less segments. Default: Error on mismatch.\n");
		printf(
				"--max : If the stereo signal is not producing the number of segments for each channel then "
						"max will choose the result with more segments. Default: Error on mismatch.\n");
		printf(
				"--fromFiltered : Triggers the module to create the segments from the filtered file instead of the original.\n");
		printf("--verbose : Triggers verbose mode to tune parameters\n");
		printf("====== \n");
		printf("press CTRL-C to continue.. \n");
		return true;
	}

	/* Getting the name of the module and the robot */
	moduleName = rf.check("name", Value("SpeechSegmenter"),
			"module name (string)").asString();
	setName(moduleName.c_str());

	robotName =
			rf.check("robot", Value("icub"), "Robot name (string)").asString();

	robotPortName = "/" + robotName + "/head";

	/* Getting port names */
	outputPortName = "/";
	outputPortName += getName(
			rf.check("outputPort", Value("/Results:o"),
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

	/* Getting parameters */
	verbose = false;
	min = false;
	max = false;
	fromFiltered = false;
	verbose = rf.check("verbose", "Verbose mode");
	min = rf.check("min", "Choose result with minimal number of segments");
	max = rf.check("max", "Choose result with maximal number of segments");
	fromFiltered = rf.check("fromFiltered",
			"Create segment files from the filtered file");

	if (min == max && min == true) {
		cerr << "Please specify either min OR max!" << endl;
		return false;
	}

	lambda = rf.check("lambda", Value(3), "lambda (int)").asInt();

	delta = rf.check("delta", Value(3), "delta (int)").asInt();

	/* Getting filter values */
	lowPassCutOffFreq = rf.check("lowpass", Value(20000),
			"Low Pass filter cut off frequency (int)").asInt();

	highPassCutOffFreq = rf.check("highpass", Value(500),
			"High Pass filter cut off frequency (int)").asInt();

	prefac =
			rf.check("prefac", Value(95), "Pre-emphasis filter factor (int)").asInt();
	mu =
			(float)rf.check("mu", Value(1.0), "Bias for threshold (float)").asDouble();

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
	controlThread = new ControlThread(&inPort, &outPort, audiofile, workdir,
			lowPassCutOffFreq, highPassCutOffFreq, prefac, lambda, delta, mu,
			min ? -1 : max ? 1 : 0, fromFiltered, verbose);
	controlThread->start();
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
//	segmentationThread->stopLoop();
//	segmentationThread->stop();
	controlThread->stopLoop();
	controlThread->stop();
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
	return true;
}

