/*
 * WriterManager.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: cdondrup
 */

#include "iCub/WriterManager.h"

bool WriterManager::init() {
	fingerprinterWriter = new ControlWriter(fingerprinter);
	segmenterWriter = new ControlWriter(segmenter);
	searchWriter = new ControlWriter(search);
	soundRecWriter = new ControlWriter(sound);
	return true;
}

void WriterManager::close() {
	delete fingerprinterWriter;
	delete segmenterWriter;
	delete searchWriter;
	delete soundRecWriter;
}

void WriterManager::operateFingerprinter(string data) {
	fingerprint_mutex.wait();
	fingerprinterWriter->write(data);
	fingerprint_mutex.post();
}

void WriterManager::operateSegmenter(string data) {
	segmenter_mutex.wait();
	segmenterWriter->write(data);
	segmenter_mutex.post();
}

void WriterManager::operateSearch(string data) {
	search_mutex.wait();
	searchWriter->write(data);
	search_mutex.post();
}

void WriterManager::operateSoundRec(string data) {
	sound_mutex.wait();
	soundRecWriter->write(data);
	sound_mutex.post();
}
