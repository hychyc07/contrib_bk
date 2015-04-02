/*
 * DataCollector.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef DATACOLLECTOR_H_
#define DATACOLLECTOR_H_

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <utility>

using namespace std;

class DataCollector {
private:
	static DataCollector *instance;
	list<pair<string, string> > loomingData;
	map<string, list<pair<string, string> > > objectData;
	list<string> soundRecBuffer;
	list<string> fingerprinterBuffer;
	list<string> segmenterBuffer;
	list<string> segmentBuffer;
	list<string> fingerSegmentBuffer;
	list<pair<string, list<pair<string, float> > > > searchBuffer;
	string soundCurrent, fingerCurrent, segmenterCurrent, segmentCurrent,
			fingerSegCurrent;
	pair<string, list<pair<string, float> > > searchCurrent;
	bool looming;
	bool right;
	bool left;
	bool interrupted;
	int sound_mutex, search_mutex, seg_mutex;
	int segmentCounter;

	string popFirstElement(list<string>* buffer);
	string getFirstElement(list<string> buffer);
	pair<string, list<pair<string, float> > > popFirstElement(
			list<pair<string, list<pair<string, float> > > > * buffer);
	pair<string, list<pair<string, float> > > getFirstElement(
			list<pair<string, list<pair<string, float> > > > buffer);
protected:
	DataCollector() :
			interrupted(false), looming(false), right(false), left(false), soundCurrent(
					""), fingerCurrent(""), segmenterCurrent(""), segmentCurrent(
					""), segmentCounter(0), fingerSegCurrent("") {
	}
public:
	static DataCollector* getInstance() {
		if (instance == 0)
			instance = new DataCollector();
		return instance;
	}
	string getSoundRecElement() {
		sound_mutex = 2;
		soundCurrent = getFirstElement(soundRecBuffer);
		return soundCurrent;
	}
	string getFingerprinterElement() {
		fingerCurrent = getFirstElement(fingerprinterBuffer);
		return fingerCurrent;
	}
	string getSegmenterElement() {
		segmenterCurrent = getFirstElement(segmenterBuffer);
		return segmenterCurrent;
	}
	string getSegment() {
		search_mutex = 1;
		list<string>::iterator it = segmentBuffer.begin();
		for (int i = 0; i < segmentBuffer.size() - segmentCounter; i++)
			++it;
		segmentCurrent = *it;
		return segmentCurrent;
	}
	string getFingerSegmentElement() {
		seg_mutex = 1;
		fingerSegCurrent = getFirstElement(fingerSegmentBuffer);
		return fingerSegCurrent;
	}
	pair<string, list<pair<string, float> > > getSearchElement() {
		searchCurrent = getFirstElement(searchBuffer);
		return searchCurrent;
	}
	string popSoundRecElement() {
		return popFirstElement(&soundRecBuffer);
	}
	string popFingerprinterElement() {
		return popFirstElement(&fingerprinterBuffer);
	}
	string popSegmenterElement() {
		return popFirstElement(&segmenterBuffer);
	}
	string popSegment() {
		--segmentCounter;
		return ""; //popFirstElement(&segmentBuffer);
	}
	string popFingerSegmentElement() {
		return popFirstElement(&fingerSegmentBuffer);
	}
	pair<string, list<pair<string, float> > > popSearchElement() {
		return popFirstElement(&searchBuffer);
	}
	bool hasSoundRecElement() {
		return soundRecBuffer.size() > 0;
	}
	bool hasFingerprinterElement() {
		return fingerprinterBuffer.size() > 0;
	}
	bool hasSegmenterElement() {
		return segmenterBuffer.size() > 0;
	}
	bool hasSegment() {
		return segmentCounter > 0;
	}
	bool hasFingerSegmentElement() {
		return fingerSegmentBuffer.size() > 0;
	}
	bool hasSearchElement() {
		return searchBuffer.size() > 0;
	}
	void addLoomingData(string name, string color) {
		loomingData.push_back(pair<string, string>(name, color));
	}
	void addObjectData(string filename) {
		objectData[filename] = loomingData;
		loomingData.clear();
	}
	void addSoundRecElement(string element) {
		soundRecBuffer.push_back(element);
		addObjectData(element);
	}
	void addFingerprinterElement(string element) {
		fingerprinterBuffer.push_back(element);
		if (element == soundCurrent)
			sound_mutex--;
		if (element == fingerSegCurrent)
			seg_mutex--;
	}
	void addSegmenterElement(string element) {
		segmenterBuffer.push_back(element);
		if (element == soundCurrent)
			sound_mutex--;
	}
	void addSegment(string element) {
		segmentBuffer.push_back(element);
		segmentCounter = segmentBuffer.size();
		clearSearchBuffer();
	}
	void addFingerSegmentElement(string element) {
		fingerSegmentBuffer.push_back(element);
	}
	void addSearchElement(string element, list<pair<string, float> > results) {
		searchBuffer.push_back(
				pair<string, list<pair<string, float> > >(element, results));
		if (element == segmentCurrent)
			search_mutex--;
		cout << searchBuffer.size() << endl;
	}
	void clearSearchBuffer() {
		searchBuffer.clear();
	}
	bool isLooming() const {
		return looming;
	}
	void setLooming(bool looming) {
		this->looming = looming;
		cout << "looming: " << isLooming() << endl;
	}
	bool isLeft() const {
		return left;
	}
	void setLeft(bool left);
	bool isRight() const {
		return right;
	}
	void setRight(bool right);

	bool waitForProcessing();
	bool waitForSegmentFingerprinting();
	bool waitForSearch();
	void interrupt();
};

#endif /* DATACOLLECTOR_H_ */
