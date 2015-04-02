/*
 * WriterManager.h
 *
 *  Created on: Aug 28, 2012
 *      Author: cdondrup
 */

#ifndef WRITERMANAGER_H_
#define WRITERMANAGER_H_

#include <string>

#include <yarp/os/all.h>

#include "iCub/ControlWriter.h"

using namespace std;
using namespace yarp::os;

class WriterManager {
private:
	ControlWriter *fingerprinterWriter;
	ControlWriter *segmenterWriter;
	ControlWriter *searchWriter;
	ControlWriter *soundRecWriter;
	Port *fingerprinter, *segmenter, *sound, *search;

	Semaphore fingerprint_mutex, segmenter_mutex, search_mutex, sound_mutex;

	bool init();
	void close();

public:
	WriterManager(Port *fingerprinter, Port *segmenter, Port *sound,
			Port *search) :
			fingerprinter(fingerprinter), segmenter(segmenter), sound(sound), search(
					search), fingerprint_mutex(1), segmenter_mutex(1), search_mutex(
					1), sound_mutex(1) {
		init();
	}
	~WriterManager() {
		close();
	}
	void operateFingerprinter(string data);
	void operateSegmenter(string data);
	void operateSearch(string data);
	void operateSoundRec(string data);

};

#endif /* WRITERMANAGER_H_ */
