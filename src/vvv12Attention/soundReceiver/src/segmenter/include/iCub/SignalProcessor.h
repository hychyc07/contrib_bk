/*
 * SignalProcessor.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cdondrup
 */

#ifndef SIGNALPROCESSOR_H_
#define SIGNALPROCESSOR_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <iomanip>

#include <fftw3.h>

#include "iCub/BandPassFilter.h"
#include "iCub/LowPassFilter.h"
#include "iCub/HighPassFilter.h"

#define PREEMPHASIS_FACTOR 0.95
#define FRAMEDURATION 25
#define FRAMESIZE 1024
#define OVERLAP FRAMESIZE/4

using namespace std;

/**
 * This class is designed to preprocess an audio file for segmentation.
 */
class SignalProcessor {
private:
	fftw_plan plan, iplan;
	int nwindows, lowPassFreq, highPassFreq;
	LowPassFilter* lowPassFilter;
	HighPassFilter* highPassFilter;
	BandPassFilter* bandPassFilter;
	bool stereo;
	float* leftFiltered;
	float* rightFiltered;
	float* leftOriginal;
	float* rightOriginal;
	float* original;
	float* filtered;
	bool verbose;
	vector<fftw_complex*> cep;
	fftw_complex * out;
	double * in;
	fftw_complex* iin;
	double* iout;
	float window[FRAMESIZE + 1], iwindow[FRAMESIZE + 1];

	/**
	 * Runs a pre-emphasis filter over the audio file to reduce the effects of
	 * the glottal pulses and radiation impedances.
	 * y(n) = x(n) - PREEMPHASIS_FACTOR * x(n-1)
	 * Original data will be overridden.
	 * @param wave The audio data
	 * @param arraysize The size of the wave array
	 * @return The filtered audio data
	 */
	float* preemphasisFilter(float* wave, int arraysize);

	/**
	 * Runs a windowed DFT over the audio data using fftw3. The window size is FRAMESIZE
	 * with an overlap of OVERLAP. The resulting data is a fftw_complex array of the size
	 * of FRAMESIZE / 2 + 1 because the data is mirrored on the nyquist.
	 * @param wave The audio date to transform
	 * @return a vector of the transformed windows as complex value arrays
	 */
	vector<fftw_complex*> windowedFFT(float* wave, int size);

	/**
	 * Inverses the windowed DFT with overlap-add to recreate the audio data.
	 * @param cepstrum A vector of the complex windows (return value of windowedFFT())
	 * @return The audio file as real data
	 */
	float* inverseWindowedFFT(vector<fftw_complex*> cepstrum, int size);

	/**
	 * Decomposes a stereo wave file into two arrays of data. One for each channel.
	 * @param wave The stereo data to decompose
	 * @return a two dimensional array (result[2][FileHandler::getFrames()]) containing
	 * the audio data
	 */
//	float** decomposeStereo(float* wave);
	/**
	 * Mixes the left and right audio channel to create one stereo file.
	 * @param wave The two dimensional array created by decomposeStereo()
	 * @return The mixed array of both channels
	 */
//	int16_t* recomposeStereo(int16_t** wave, int size);
	/**
	 * Creates a Hann window which is used by the windowedFFT().
	 * The buffer has to be muliplied with the data.
	 * @param windowLength The length of the desired window
	 * @param buffer An Array to write the window factors to
	 */
	void hanning(int windowLength, float* buffer);

public:
	/**
	 * Creates a new SignalProcessor.
	 * @param lowPassFreq The cut off frequency for the LowPassFilter used by the BandPassFilter
	 * @param highPassFreq The cut off frequency for the HighPassFilter used by the BandPassFilter
	 */
	SignalProcessor(int lowPassFreq);
	~SignalProcessor();

	/**
	 * Calls the FileHandler::read() function to load an audio file an preprocesses it by calling all
	 * the necessary private functions of this class.
	 * @param fileHandler The FileHandler handling the audio file to preprocess
	 * @return true if successful
	 */
	float* process(float* signal, int size);

	/**
	 * @return The filtered original audio data
	 */
	float* getFiltered() const {
		return filtered;
	}

	/**
	 * @param filtered The filtered original audio data
	 */
	void setFiltered(float* filtered) {
		this->filtered = filtered;
	}

	/**
	 * @return The untouched original audio data
	 */
	float* getOriginal() const {
		return original;
	}

	/**
	 * @param original The untouched original audio data
	 */
	void setOriginal(float* original) {
		this->original = original;
	}

	/**
	 * @return The filtered left channel audio data of a stereo file
	 */
	float* getLeftFiltered() const {
		return leftFiltered;
	}

	/**
	 * @param leftFiltered The filtered left channel audio data of a stereo file
	 */
	void setLeftFiltered(float* leftFiltered) {
		this->leftFiltered = leftFiltered;
	}

	/**
	 * @return The original left channel audio data of a stereo file
	 */
	float* getLeftOriginal() const {
		return leftOriginal;
	}

	/**
	 * @param leftOriginal The original left channel audio data of a stereo file
	 */
	void setLeftOriginal(float* leftOriginal) {
		this->leftOriginal = leftOriginal;
	}

	/**
	 * @return The filtered right channel audio data of a stereo file
	 */
	float* getRightFiltered() const {
		return rightFiltered;
	}

	/**
	 * @param rightFiltered The filtered right channel audio data of a stereo file
	 */
	void setRightFiltered(float* rightFiltered) {
		this->rightFiltered = rightFiltered;
	}

	/**
	 * @return The original right channel audio data of a stereo file
	 */
	float* getRightOriginal() const {
		return rightOriginal;
	}
	/**
	 * @param rightOriginal The original right channel audio data of a stereo file
	 */
	void setRightOriginal(float* rightOriginal) {
		this->rightOriginal = rightOriginal;
	}

	/**
	 * @return true if file is stereo (channels == 2)
	 */
	bool isStereo() const {
		return stereo;
	}
};

#endif /* SIGNALPROCESSOR_H_ */
