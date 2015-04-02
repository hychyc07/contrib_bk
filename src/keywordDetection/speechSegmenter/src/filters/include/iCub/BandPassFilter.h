/*
 * BandPassFilter.h
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#ifndef BANDPASSFILTER_H_
#define BANDPASSFILTER_H_

#include <vector>
#include <iostream>

#include <fftw3.h>

#include "iCub/Filter.h"
#include "iCub/LowPassFilter.h"
#include "iCub/HighPassFilter.h"

using namespace std;

/**
 * @brief Combines a LowPassFilter and a HighPassFilter to create a BandPassFilter.
 *
 * A BandPassFilter removes all low frequencies (background noise) and all high frequencies
 * (high frequency noise and speech harmonics due to sampling and nonlinearity of the recording
 * device). This is necessary for the entropy and threshold calculation.
 * This Filter is designed to be used on a DFTed window of the original wave file.
 */
class BandPassFilter: public Filter {
private:
	int size;
	float* filter;

public:
	/**
	 * Constructs a new BandPassFilter
	 * @param size The size of the filter which will be constructed. Should match the size of the
	 * data to filter
	 * @param sampleFreq The sample frequency (should be 44100)
	 * @param lowCutoffFreq Every frequency over the low cut off is removed (has to be > highCutoffFreq)
	 * @param highCutoffFreq Every frequency below the high cut off is removed (has to be < lowCutoffFreq)
	 */
	BandPassFilter(int size, int sampleFreq, int lowCutoffFreq, int highCutoffFreq);

	/**
	 * Deletes the constructed filter array.
	 */
	~BandPassFilter();
};

#endif /* BANDPASSFILTER_H_ */
