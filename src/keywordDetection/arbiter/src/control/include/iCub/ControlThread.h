/*
 * ControlThread.h
 *
 *  Created on: Aug 28, 2012
 *      Author: cdondrup
 */

#ifndef CONTROLTHREAD_H_
#define CONTROLTHREAD_H_

#include <yarp/os/all.h>

#include "iCub/DataCollector.h"
#include "iCub/WriterManager.h"

using namespace std;
using namespace yarp::os;

class ControlThread: public Thread {
private:

	/* class variables */
	bool stoploop;
	WriterManager *writerManager;

	/* thread parameters: */

public:

	/* class methods */

	ControlThread(WriterManager *writerManager) :
			writerManager(writerManager) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
	}

	void run() {
		cout << " ControleThread" << endl;
		while (!stoploop) {
			if (DataCollector::getInstance()->hasSoundRecElement()) {
				cout << "Start preprocessing for "
						<< DataCollector::getInstance()->getSoundRecElement()
						<< endl;
				writerManager->operateFingerprinter(
						DataCollector::getInstance()->getSoundRecElement());
				writerManager->operateSegmenter(
						DataCollector::getInstance()->getSoundRecElement());
				if (!DataCollector::getInstance()->waitForProcessing())
					cout << "Processing interrupted!" << endl;
				cout << "Finished preprocessing." << endl;
				DataCollector::getInstance()->popSoundRecElement();
				continue;
			}
			if (DataCollector::getInstance()->hasFingerSegmentElement()) {
				cout << "Start fingerprinting for segment "
						<< DataCollector::getInstance()->getFingerSegmentElement()
						<< endl;
				writerManager->operateFingerprinter(
						DataCollector::getInstance()->getFingerSegmentElement());
				if(!DataCollector::getInstance()->waitForSegmentFingerprinting())
					cout<<"Segment fingerprinting interrupted"<<endl;
				DataCollector::getInstance()->popFingerSegmentElement();
				continue;
			}
			if (DataCollector::getInstance()->hasSegment()) {
				cout << "Start searching for "
						<< DataCollector::getInstance()->getSegment() << endl;
				writerManager->operateSearch(
						DataCollector::getInstance()->getSegment());
				if (!DataCollector::getInstance()->waitForSearch())
					cout << "Search interrupted!" << endl;
				cout << "Finished searching." << endl;
				DataCollector::getInstance()->popSegment();
				continue;
			}
		}
	}

	/**
	 * Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		DataCollector::getInstance()->interrupt();
	}
};

#endif /* CONTROLTHREAD_H_ */
