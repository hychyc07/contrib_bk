/*
 * SoundRecReader.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef SOUNDRECREADER_H_
#define SOUNDRECREADER_H_

#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class SoundRecReader: public Thread {
private:

	/* class variables */
	bool stoploop;
	Port* in;

	/* thread parameters: */

public:

	/* class methods */

	SoundRecReader(Port *in) :
			in(in) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
//	   cout<<"  SoundRecReader"<<endl;
	}

	/**
	 * Reads from the Port instance given on creation.
	 * And sends the gathered data to the DataCollector.
	 */
	void run() {
		cout << " SoundRecReader" << endl;
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			if (bot.size() > 0) {
				DataCollector::getInstance()->addSoundRecElement(
						string(bot.get(0).asString().c_str()));
			}
		}
	}

	/**
	 * Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		in->interrupt();
	}
};

#endif /* SOUNDRECREADER_H_ */
