/*
 * HighPassFilter.h
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#ifndef HIGHPASSFILTER_H_
#define HIGHPASSFILTER_H_

#include <vector>

#include <fftw3.h>

#include "iCub/Filter.h"

using namespace std;

/**
 * @brief Creates a HighPassFilter.
 *
 * A HighPassFilter removes all high frequencies
 * (high frequency noise and speech harmonics due to sampling and nonlinearity of the recording
 * device). This is necessary for the construction of the BandPassFilter.
 * This Filter is designed to be used on a DFTed window of the original wave file.
 */
class HighPassFilter: public Filter {
private:
	int size;
	float* filter;

public:
	/**
	 * Constructs a new HighPassFilter
	 * @param size The size of the filter which will be constructed. Should match the size of the
	 * data to filter
	 * @param sampleFreq The sample frequency (should be 44100)
	 * @param cutoffFreq Every frequency above the high cut off is removed
	 */
	HighPassFilter(int size, int sampleFreq, int cutoffFreq);

	/**
	 * Deletes the constructed filter array.
	 */
	~HighPassFilter();
};

#endif /* HIGHPASSFILTER_H_ */
