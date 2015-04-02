/*
 * LowPassFilter.h
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

#include <vector>

#include <fftw3.h>

#include "iCub/Filter.h"

using namespace std;

/**
 * @brief Creates a LowPassFilter.
 *
 * A LowPassFilter removes all low frequencies (background noise).
 * This is necessary for the construction of the BandPassFilter.
 * This Filter is designed to be used on a DFTed window of the original wave file.
 */
class LowPassFilter: public Filter {
private:
	int size;
	float* filter;

public:
	/**
	 * Constructs a new LowPassFilter
	 * @param size The size of the filter which will be constructed. Should match the size of the
	 * data to filter
	 * @param sampleFreq The sample frequency (should be 44100)
	 * @param cutoffFreq Every frequency below the low cut off is removed
	 */
	LowPassFilter(int size, int sampleFreq, int cutoffFreq);
	~LowPassFilter();
};

#endif /* LOWPASSFILTER_H_ */
