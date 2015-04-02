/*
 * SignalProcessor.cpp
 *
 *  Created on: Jul 5, 2012
 *      Author: cdondrup
 */

#include "iCub/SignalProcessor.h"

using namespace std;

SignalProcessor::SignalProcessor(int lowPassFreq) {
	this->lowPassFreq = lowPassFreq;
	highPassFilter = new HighPassFilter(FRAMESIZE / 2 + 1, 48000, lowPassFreq);
	//Allocating memory

	in = (double *) fftw_malloc(sizeof(double) * FRAMESIZE);
	out = (fftw_complex *) fftw_malloc(
			sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));
	iin = (fftw_complex *) fftw_malloc(
			sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));
	iout = (double *) fftw_malloc(sizeof(double) * (2 * (FRAMESIZE / 2 + 1)));

	hanning(FRAMESIZE, window);
	hanning(FRAMESIZE, iwindow);
	for (int i = 0; i <= FRAMESIZE; i++)
		iwindow[i] *= (2.0f / 3.0f);

	//Creating FFT execution plan
	plan = fftw_plan_dft_r2c_1d(FRAMESIZE, in, out, FFTW_ESTIMATE);
	iplan = fftw_plan_dft_c2r_1d(FRAMESIZE, iin, iout, FFTW_ESTIMATE);
}

SignalProcessor::~SignalProcessor() {
	fftw_destroy_plan(plan);
	fftw_free(in);
	fftw_free(out);

	fftw_destroy_plan(iplan);
	fftw_free(iin);
	fftw_free(iout);
}

float* SignalProcessor::process(float* signal, int size) {
	cep = windowedFFT(signal, size);
//	lowPassFilter = new LowPassFilter(size, 48000, lowPassFreq);
	cep = highPassFilter->filterSignal(cep);
	return inverseWindowedFFT(cep, size);
}

float* SignalProcessor::preemphasisFilter(float* wave, int arraysize) {
	for (int i = 1; i < arraysize; i++) {
		wave[i] -= PREEMPHASIS_FACTOR * wave[i - 1];
	}
	return wave;
}

vector<fftw_complex*> SignalProcessor::windowedFFT(float* wave, int size) {
	vector<fftw_complex*> result;

	//Main loop over the complete data
	for (int i = 0; i < size - FRAMESIZE; i += OVERLAP) {
		//Creating the data window and overlaying it with the Hann window
		for (int j = 0; j < FRAMESIZE; j++)
			in[j] = wave[i + j] * window[j];

		//Execute FFT on current window
		fftw_execute(plan);

		//Normalizing complex results of FFT
		for (unsigned int k = 0; k < FRAMESIZE / 2 + 1; k++) {
			out[k][0] /= (float) FRAMESIZE;
			out[k][1] /= (float) FRAMESIZE;
		}

		//Writing result to an array and adding it to the result vector
		fftw_complex* tmp = (fftw_complex *) fftw_malloc(
				sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));
		for (int k = 0; k < FRAMESIZE / 2 + 1; k++) {
			tmp[k][0] = out[k][0];
			tmp[k][1] = out[k][1];
		}
		result.push_back(tmp);
	}

	return result;
}

void SignalProcessor::hanning(int windowLength, float *buffer) {
	//To ensure that there is only one peek in the symmetrical window
	//we hav to create a "half window" which holds all values from
	//0 to one created bey the formula below
	int halflen = windowLength / 2;
	float halfwin[halflen + 1];
	for (int i = 0; i <= halflen; i++) {
		halfwin[i] = 0.5 * (1 + cos(M_PI * i / (halflen)));
	}
	//After that the first windowLength/2+1 elements are pasted into the buffer
	int j = 0;
	for (int i = halflen; i >= 0; i--) {
		buffer[j] = halfwin[i];
		j++;
	}
	//Then the other half of the window is filled by the reversed half window.
	//The reversed half window starts with 1 and overwrites the 1 that was the last
	//element of the not reversed half window. So we only have one 1 in the whole window
	for (int i = 0; i < halflen; i++) {
		buffer[i + halflen] = halfwin[i];
	}
}

float* SignalProcessor::inverseWindowedFFT(vector<fftw_complex*> cepstrum,
		int size) {
	//Create an array to hold the reconstructed real data
	float* result = new float[size];
	//and initialize it with 0's
	for (int i = 0; i < size; i++)
		result[i] = 0.0;

	//Execute the plan for every complex window
	for (int i = 0; i < cepstrum.size(); i++) {
		for (int j = 0; j < FRAMESIZE / 2 + 1; j++) {
			iin[j][0] = cepstrum[i][j][0];
			iin[j][1] = cepstrum[i][j][1];
		}
		fftw_execute(iplan);

		fftw_free(cepstrum[i]);

		//Write the resulting real data to result.
		//Values will be summed where the windows overlap.
		//And the real data window is multiplied with the inversed Hann window.
		for (int k = 0; k < (2 * (FRAMESIZE / 2 + 1)); k++)
			result[i * OVERLAP + k] += (iout[k] * iwindow[k]);
	}
	return result;
}
