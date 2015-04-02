/*
 * DataCollector.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#include "iCub/DataCollector.h"

DataCollector* DataCollector::instance = 0;

string DataCollector::popFirstElement(list<string>* buffer) {
	string result = buffer->front();
	buffer->pop_front();
	return result;
}

string DataCollector::getFirstElement(list<string> buffer) {
	return buffer.front();
}

pair<string,list<pair<string,float> > > DataCollector::popFirstElement(list<pair<string,list<pair<string,float> > > >* buffer) {
	pair<string,list<pair<string,float> > > result = buffer->front();
	buffer->pop_front();
	return result;
}

pair<string,list<pair<string,float> > > DataCollector::getFirstElement(list<pair<string,list<pair<string,float> > > > buffer) {
	return buffer.front();
}

void DataCollector::setLeft(bool left) {
	this->left = left;
	if (isLeft()) {
		if (!isLooming())
			setLooming(true);
	} else {
		if (!isRight() && isLooming()) {
			setLooming(false);
		}
	}
}

void DataCollector::setRight(bool right) {
	this->right = right;
	if (isRight()) {
		if (!isLooming())
			setLooming(true);
	} else {
		if (!isLeft() && isLooming()) {
			setLooming(false);
		}
	}
}

bool DataCollector::waitForProcessing() {
	interrupted = false;
	while (!interrupted) {
		if (sound_mutex == 0) {
			soundCurrent = "";
			break;
		}
	}
	return !interrupted;
}

bool DataCollector::waitForSegmentFingerprinting() {
	interrupted = false;
	while (!interrupted) {
		if (seg_mutex == 0) {
			fingerSegCurrent = "";
			break;
		}
	}
	return !interrupted;
}

bool DataCollector::waitForSearch() {
	interrupted = false;
	while (!interrupted) {
		if (search_mutex == 0)
			segmentCurrent = "";
			break;
	}
	return !interrupted;
}

void DataCollector::interrupt() {
	interrupted = true;
}
