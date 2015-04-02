/*
 * SegmenterReader.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef SEGMENTERREADER_H_
#define SEGMENTERREADER_H_

#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class SegmenterReader: public Thread {
private:

	/* class variables */
	bool stoploop;
	Port* in;

	/* thread parameters: */

public:

	/* class methods */

	SegmenterReader(Port *in) :
			in(in) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
//	   cout<<"  SegmentReader"<<endl;
	}

	/**
	 * Reads from the Port instance given on creation.
	 * And sends the gathered data to the DataCollector.
	 */
	void run() {
		cout << " SegmentReader" << endl;
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			if (bot.size() > 0) {
				switch (bot.get(0).asVocab()) {
				case VOCAB4('O','R','I','G'):
					DataCollector::getInstance()->addSegmenterElement(
							string(bot.get(1).asString().c_str()));
					break;
				case VOCAB3('S','E','G'):
					DataCollector::getInstance()->addSegment(
							string(bot.get(1).asString().c_str()));
					DataCollector::getInstance()->addFingerSegmentElement(
							string(bot.get(1).asString().c_str()));
					break;
				}
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

#endif /* SEGMENTERREADER_H_ */
