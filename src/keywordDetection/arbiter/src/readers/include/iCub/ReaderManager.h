/*
 * ReaderManager.h
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#ifndef READERMANAGER_H_
#define READERMANAGER_H_

#include <string>

#include <yarp/os/all.h>

#include "iCub/LoomingReader.h"
#include "iCub/SearchReader.h"
#include "iCub/SoundRecReader.h"
#include "iCub/SegmenterReader.h"
#include "iCub/FingerprinterReader.h"

using namespace std;
using namespace yarp::os;

class ReaderManager {
private:
	Port *loomingIn, *searchIn, *soundIn, *segmenterIn, *fingerprinterIn;
	LoomingReader* loomingReader;
	SearchReader* searchReader;
	SoundRecReader* soundReader;
	SegmenterReader* segmentReader;
	FingerprinterReader* fingerprintReader;

	bool init();
	void close();
public:
	ReaderManager(Port *loomingIn, Port *searchIn, Port *soundIn,
			Port *segmenterIn, Port *fingerprinterIn) :
			loomingIn(loomingIn), searchIn(searchIn), soundIn(soundIn), segmenterIn(
					segmenterIn), fingerprinterIn(fingerprinterIn) {
		init();
	}

	~ReaderManager() {
		close();
	}
};

#endif /* READERMANAGER_H_ */
