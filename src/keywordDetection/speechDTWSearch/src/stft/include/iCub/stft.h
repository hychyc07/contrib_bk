/*
 * stft.h
 *
 *  Created on: Oct 29, 2012
 *      Author: cdondrup
 */

#ifndef STFT_H_
#define STFT_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <cstdlib>

#include <fftw3.h>

#define FRAMESIZE 512
#define OVERLAP FRAMESIZE/4
#define STEP FRAMESIZE-OVERLAP

using namespace std;

class stft {
private:

public:
	/**
	 * Runs a windowed DFT over the audio data using fftw3. The window size is FRAMESIZE
	 * with an overlap of OVERLAP. The resulting data is a fftw_complex array of the size
	 * of FRAMESIZE / 2 + 1 because the data is mirrored on the nyquist.
	 * @param wave The audio date to transform
	 * @return a vector of the transformed windows as complex value arrays
	 */
	static vector<fftw_complex*> windowedFFT(float* wave, int frames) {
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
		fftw_plan plan = fftw_plan_dft_r2c_1d(FRAMESIZE, in, out, FFTW_ESTIMATE);

		//Main loop over the complete data
		for (int i = 0; i < frames - FRAMESIZE; i += OVERLAP) {
			//Creating the data window and overlaying it with the Hann window
			for (int j = 0; j < FRAMESIZE; j++)
				in[j] = wave[i + j] * window[j];

			//Execute FFT on current window
			fftw_execute(plan);

			//Normalizing complex results of FFT
//			for (unsigned int k = 0; k < FRAMESIZE / 2 + 1; k++) {
//				out[k][0] /= (float) FRAMESIZE;
//				out[k][1] /= (float) FRAMESIZE;
//			}

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

	/**
	 * Creates a Hann window which is used by the windowedFFT().
	 * The buffer has to be muliplied with the data.
	 * @param windowLength The length of the desired window
	 * @param buffer An Array to write the window factors to
	 */
	static void hanning(int windowLength, float* buffer) {
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
};

#endif /* STFT_H_ */
