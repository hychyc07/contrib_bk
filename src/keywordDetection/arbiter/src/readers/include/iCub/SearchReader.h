/*
 * SearchReader.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef SEARCHREADER_H_
#define SEARCHREADER_H_

#include <list>
#include <string>
#include <iostream>
#include <utility>
#include <cstring>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class SearchReader: public Thread {
private:

	/* class variables */
	bool stoploop;
	Port* in;

	/* thread parameters: */

public:

	/* class methods */

	SearchReader(Port *in) :
			in(in) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
//	   cout<<"  SearchReader"<<endl;
	}

	/**
	 * Reads from the Port instance given on creation.
	 * And sends the gathered data to the DataCollector.
	 */
	void run() {
		cout << " SearchReader" << endl;
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			if (bot.size() > 0) {
				switch (bot.get(0).asVocab()) {
				case VOCAB3('R','E','S'): {
					Bottle *results = bot.get(1).asList();
					list<pair<string, float> > resultlist;
					for (int i = 0; i < results->size(); i += 2) {
						if (strcmp(results->get(i).asString().c_str(),bot.get(3).asString().c_str())==0)
							continue;
						resultlist.push_back(
								pair<string, float>(
										string(
												results->get(i).asString().c_str()),
										results->get(i + 1).asDouble()));
					}
					DataCollector::getInstance()->addSearchElement(
							string(bot.get(3).asString().c_str()), resultlist);
					break;
				}
				case VOCAB3('E','R','R'): {
					list<pair<string, float> > resultlist;
					DataCollector::getInstance()->addSearchElement(
							string(bot.get(3).asString().c_str()), resultlist);
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

#endif /* SEARCHREADER_H_ */
