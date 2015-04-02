/*
 * SignalProcessor.cpp
 *
 *  Created on: Jul 5, 2012
 *      Author: cdondrup
 */

#include "iCub/SignalProcessor.h"

using namespace std;

bool SignalProcessor::process(FileHandler* fileHandler) {
	//Reading the audio file
	this->fileHandler = fileHandler;
	if (!fileHandler->read())
		return false;

	if (verbose)
		cout << "Processing signal:" << endl;

	//Creating a bandpass filter
	bandPassFilter = new BandPassFilter(FRAMESIZE / 2 + 1,
			fileHandler->getSamplerate(), lowPassFreq, highPassFreq);

	if (fileHandler->getChannels() == 2) {
		stereo = true;
		if (verbose)
			cout << "Decomposing stereo file" << endl;
		float** wave = decomposeStereo(fileHandler->getWave());
		setLeftOriginal(wave[0], fileHandler->getFrames());
		setRightOriginal(wave[1], fileHandler->getFrames());

		if (verbose)
			cout << "Pre-emphasis filtering" << endl;
		preemphasisFilter(wave[0], fileHandler->getFrames());
		preemphasisFilter(wave[1], fileHandler->getFrames());

		if (verbose)
			cout << "Short term fourie transform" << endl;
		//Needed for filtering
		vector<fftw_complex*> cep1 = windowedFFT(wave[0]);
		vector<fftw_complex*> cep2 = windowedFFT(wave[1]);

		/* Filtering done here */
		if (verbose)
			cout << "Bandpass filtering" << endl;
		bandPassFilter->filterSignal(cep1);
		bandPassFilter->filterSignal(cep2);

		if (verbose)
			cout << "Inverse short term fourie transform" << endl;
		delete [] wave[0];
		delete [] wave[1];
		wave[0] = inverseWindowedFFT(cep1);
		wave[1] = inverseWindowedFFT(cep2);

		setLeftFiltered(wave[0]);
		setRightFiltered(wave[1]);

		if (verbose)
			cout << "Recomposing stereo file" << endl << endl;
		fileHandler->setWave(recomposeStereo(wave));
	} else {
		stereo = false;
		setOriginal(fileHandler->getWave());
		if (verbose)
			cout << "Pre-emphasis filtering" << endl;
		preemphasisFilter(fileHandler->getWave(), fileHandler->getArraySize());

		if (verbose)
			cout << "Short term fourie transform" << endl;
		vector<fftw_complex*> cep = windowedFFT(fileHandler->getWave());

		if (verbose)
			cout << "Bandpass filtering" << endl;
		bandPassFilter->filterSignal(cep);

		if (verbose)
			cout << "Inverse short term fourie transform" << endl;
		fileHandler->setWave(inverseWindowedFFT(cep));
	}

	if (!fileHandler->writeFilteredFile())
		return false;
	return true;
}

float** SignalProcessor::decomposeStereo(float* wave) {
	float** result = new float*[2];
	result[0] = new float[fileHandler->getFrames()];
	result[1] = new float[fileHandler->getFrames()];
	long j = 0;
	//Stereo data is just one mixed array -> left,right,left,right...
	for (long i = 0; i < fileHandler->getArraySize(); i++) {
		if (i % 2 == 0) {
			result[0][j] = wave[i];
		} else {
			result[1][j] = wave[i];
			j++;
		}
	}
	return result;
}

float* SignalProcessor::recomposeStereo(float** wave) {
	float* result = new float[fileHandler->getArraySize()];
	long j = 0;
	//Mixing two arrays into one to create stereo data
	for (long i = 0; i < fileHandler->getFrames(); i++) {
		result[j++] = wave[0][i];
		result[j++] = wave[1][i];
	}
	return result;
}

float* SignalProcessor::preemphasisFilter(float* wave, int arraysize) {
	for (int i = 1; i < arraysize; i++) {
		wave[i] -= prefac * wave[i - 1];
	}
	return wave;
}

vector<fftw_complex*> SignalProcessor::windowedFFT(float* wave) {
	//Creating a Hann window
	float window[FRAMESIZE + 1];
	hanning(FRAMESIZE, window);

	//Allocating memory
	fftw_complex * out;
	vector<fftw_complex*> result;
	double * in;
	in = (double *) fftw_malloc(sizeof(double) * FRAMESIZE);
	out = (fftw_complex *) fftw_malloc(
			sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));

	//Creating FFT execution plan
	plan = fftw_plan_dft_r2c_1d(FRAMESIZE, in, out, FFTW_ESTIMATE);

	//Main loop over the complete data
	for (int i = 0; i < fileHandler->getFrames() - FRAMESIZE; i += OVERLAP) {
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

	//Freeing resources
	fftw_destroy_plan(plan);

	fftw_free(in);
	fftw_free(out);

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

float* SignalProcessor::inverseWindowedFFT(vector<fftw_complex*> cepstrum) {
	//Create an array to hold the reconstructed real data
	float* result = new float[fileHandler->getFrames()];
	//and initialize it with 0's
	for (int i = 0; i < fileHandler->getFrames(); i++)
		result[i] = 0.0;

	//Creating a window that will reverse the effects of the Hann window.
	//If you do not reverse the windowing, it will lead to to unwanted noise.
	float window[FRAMESIZE + 1];
	hanning(FRAMESIZE, window);
	for (int i = 0; i <= FRAMESIZE; i++)
		window[i] *= (2.0f / 3.0f);

	//Allocating memory
	fftw_complex* in = (fftw_complex *) fftw_malloc(
			sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));
	double* out = (double *) fftw_malloc(
			sizeof(double) * (2 * (FRAMESIZE / 2 + 1)));

	//Creating a inversed FFT execution plan
	plan = fftw_plan_dft_c2r_1d(FRAMESIZE, in, out, FFTW_ESTIMATE);

	//Execute the plan for every complex window
	for (int i = 0; i < cepstrum.size(); i++) {
		for (int j = 0; j < FRAMESIZE / 2 + 1; j++) {
			in[j][0] = cepstrum[i][j][0];
			in[j][1] = cepstrum[i][j][1];
		}
		fftw_execute(plan);

		//Write the resulting real data to result.
		//Values will be summed where the windows overlap.
		//And the real data window is multiplied with the inversed Hann window.
		for (int k = 0; k < (2 * (FRAMESIZE / 2 + 1)); k++)
			result[i * OVERLAP + k] += (out[k] * window[k]);
	}
	return result;
}
