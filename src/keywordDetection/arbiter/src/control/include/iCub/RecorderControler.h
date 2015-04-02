/*
 * RecorderControler.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef RECORDERCONTROLER_H_
#define RECORDERCONTROLER_H_

#include <yarp/os/all.h>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;

class RecorderControler: public Thread {
private:

	/* class variables */
	bool stoploop;
	bool looming;
	Port* out;

	/* thread parameters: */

public:

	/* class methods */

	RecorderControler(Port *out) :
			out(out) {
	}
	bool threadInit() {
		stoploop = false;
		looming = false;
		return true;
	}
	void threadRelease() {
	}

	/**
	 * Reads from the Port instance given on creation.
	 * And sends the gathered data to the DataCollector.
	 */
	void run() {
		cout << " RecorderControler" << endl;
		while (!stoploop) {
			if (looming != DataCollector::getInstance()->isLooming()) {
				looming = DataCollector::getInstance()->isLooming();
				Bottle bot;
				bot.addString(looming ? "start" : "stop");
				out->write(bot);
			}
		}
	}

	/**
	 * Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		out->interrupt();
	}
};

#endif /* RECORDERCONTROLER_H_ */
