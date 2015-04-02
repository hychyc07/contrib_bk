/*
 * ReaderManager.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: cdondrup
 */

#include "iCub/ReaderManager.h"

bool ReaderManager::init() {
	loomingReader = new LoomingReader(loomingIn);
	loomingReader->start();
	searchReader = new SearchReader(searchIn);
	searchReader->start();
	fingerprintReader = new FingerprinterReader(fingerprinterIn);
	fingerprintReader->start();
	segmentReader = new SegmenterReader(segmenterIn);
	segmentReader->start();
	soundReader = new SoundRecReader(soundIn);
	soundReader->start();
	return true;
}

void ReaderManager::close() {
	loomingReader->stopLoop();
	loomingReader->stop();
	delete loomingReader;
	searchReader->stopLoop();
	searchReader->stop();
	delete searchReader;
	fingerprintReader->stopLoop();
	fingerprintReader->stop();
	delete fingerprintReader;
	segmentReader->stopLoop();
	segmentReader->stop();
	delete segmentReader;
	soundReader->stopLoop();
	soundReader->stop();
	delete soundReader;
}


