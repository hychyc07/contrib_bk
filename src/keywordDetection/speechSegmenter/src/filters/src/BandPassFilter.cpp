/*
 * BandPassFilter.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#include "iCub/BandPassFilter.h"

using namespace std;

BandPassFilter::BandPassFilter(int size, int sampleFreq, int lowCutoffFreq,
		int highCutoffFreq) {
	LowPassFilter* lpf = new LowPassFilter(size, sampleFreq, lowCutoffFreq);
	HighPassFilter* hpf = new HighPassFilter(size, sampleFreq, highCutoffFreq);
	filter = new float[size];
	//Multiply the low and high pass filter to create a band pass filter
	for (int i = 0; i < size; i++)
		filter[i] = float(lpf->getFilter()[i]) * float(hpf->getFilter()[i]);
	//Giving the filter and the size to the super class Filter
	setFilter(filter);
	setSize(size);
}

BandPassFilter::~BandPassFilter() {
	delete[] filter;
}
