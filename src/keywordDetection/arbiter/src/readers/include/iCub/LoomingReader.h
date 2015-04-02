/*
 * LoomingReader.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef LOOMINGREADER_H_
#define LOOMINGREADER_H_

#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class LoomingReader: public Thread {
private:

	/* class variables */
	bool stoploop;
	Port* in;

	/* thread parameters: */

public:

	/* class methods */

	LoomingReader(Port *in) :
			in(in) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
//	   cout<<"  LoomingReader"<<endl;
	}

	/**
	 * Reads from the Port instance given on creation.
	 * And sends the gathered data to the DataCollector.
	 */
	void run() {
		cout << " LoomingReader" << endl;
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			if (bot.size() > 0) {
				switch (bot.get(0).asVocab()) {
				case VOCAB4('s','t','a','r'):
					switch (bot.get(1).asVocab()) {
					case VOCAB4('l','e','f','t'):
						DataCollector::getInstance()->setLeft(true);
						break;
					case VOCAB4('r','i','g','h'):
						DataCollector::getInstance()->setRight(true);
						break;
					}
					break;
				case VOCAB4('s','t','o','p'):
					switch (bot.get(1).asVocab()) {
					case VOCAB4('l','e','f','t'):
						DataCollector::getInstance()->setLeft(false);
						break;
					case VOCAB4('r','i','g','h'):
						DataCollector::getInstance()->setRight(false);
						break;
					}
					break;
				}
				for (int i = 0; i < bot.size(); i++) {
					if (bot.get(i).asVocab() == VOCAB3('O','B','J')) {
						DataCollector::getInstance()->addLoomingData(
								string(bot.get(i + 1).asString().c_str()),
								string(bot.get(i + 1).asString().c_str()));
						break;
				}
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

#endif /* LOOMINGREADER_H_ */
