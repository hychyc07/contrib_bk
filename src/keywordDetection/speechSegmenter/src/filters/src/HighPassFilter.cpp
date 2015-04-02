/*
 * HighPassFilter.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */


#include "iCub/HighPassFilter.h"

using namespace std;

HighPassFilter::HighPassFilter(int size, int sampleFreq, int cutoffFreq) {
	this->size = size;
	filter = new float[size];
	for (int i = 0; i < size; i++) {
		if (i < (size * cutoffFreq / sampleFreq)) {
			// passband
			filter[i] = 0.0;
		} else {
			// stopband
			filter[i] = 1.0;
		}
	}
	//Giving the filter and the size to the super class Filter
	setFilter(filter);
	setSize(size);
}

HighPassFilter::~HighPassFilter() {
	delete[] filter;
}

