/*
 * Fingerprinter.h
 *
 *  Created on: Jun 22, 2012
 *      Author: Christian Dondrup
 */

#ifndef FINGERPRINTER_H_
#define FINGERPRINTER_H_

#include <iostream>
#include <dirent.h>
#include <cstdlib>
#include <fstream>

#include <yarp/os/all.h>

#include <musicretr-1.0/sigproc.h>
#include <musicretr-1.0/keypoint.h>

#include "iCub/StringConverter.h"
#include "iCub/DatabaseBuilder.h"

using namespace std;
using namespace yarp::os;

/**
 * This class represents the work thread of the program.
 * It creates fingerprint files for all the given audio files.
 */
class Fingerprinter: public Thread {
private:
	bool stoploop, overwrite;
	string audiofile, filterfile, workdir;
	vector<Filter> filters;
	Port *control, *result;

	/**
	 * @brief Creates a name for the .keys files.
	 *
	 * This name consists of the current system time in seconds.milliseconde and the .keys suffix.
	 * @param workdir The directory where the .kyes files should be stored
	 * @param filename The original filename
	 */
	string createFilename(string workdir, string filename, int num = 0);

	/**
	 * @brief Creates a fingerprint for the given wave file.
	 * @param file The path to a wave file
	 */
	bool fingerprint(char* file);

	bool createDatabase(string workdir);

public:
	/**
	 * @brief Creates a new Fingerprinter object.
	 * @param audiofile The path to a wave file or the path to a list of wave files (.txt)
	 * @param filters The path to the filter descriptor file
	 * @param workdir The path to a directory where everything should be stored
	 * @param overwrite Iff true any file with the same name in the working directory
	 * will be overwritten
	 */
	Fingerprinter(string audiofile, string filters, string workdir,
			Port *control, Port *result, bool overwrite) :
			audiofile(audiofile), filterfile(filters), workdir(workdir), overwrite(
					overwrite), control(control), result(result) {
	}

	bool threadInit();
	void threadRelease();

	/**
	 * @brief Starts the main loop of the Thread.
	 *
	 * Checks if a wave for or a list of files is given and converts them to .keys files and saves them to the
	 * working directory.
	 */
	void run();

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		control->interrupt();
		result->interrupt();
	}
};

#endif /* FINGERPRINTER_H_ */
