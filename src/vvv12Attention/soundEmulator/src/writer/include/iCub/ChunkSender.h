/*
 * ChunkSender.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cdondrup
 */

#ifndef CHUNKSENDER_H_
#define CHUNKSENDER_H_

#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/FileReader.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class ChunkSender: public Thread {
private:
	bool stoploop;
	FileReader fileReader;

	BufferedPort<Sound>* out;
	string filename;


public:
	ChunkSender(BufferedPort<Sound>* out, string filepath) :
			out(out), filename(filepath) {
	}
	bool send();

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
	}
};

#endif /* CHUNKSENDER_H_ */
