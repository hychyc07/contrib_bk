/*
 * istft.h
 *
 *  Created on: Oct 29, 2012
 *      Author: cdondrup
 */

#ifndef ISTFT_H_
#define ISTFT_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <cstdlib>

#include <fftw3.h>

#include "iCub/stft.h"

class istft {
private:

public:
	/**
	 * Inverses the windowed DFT with overlap-add to recreate the audio data.
	 * @param cepstrum A vector of the complex windows (return value of windowedFFT())
	 * @return The audio file as real data
	 */
	static float* inverseWindowedFFT(vector<fftw_complex*> cepstrum,
			int frames) {
		//Create an array to hold the reconstructed real data
		float* result = new float[frames];
		//and initialize it with 0's
		for (int i = 0; i < frames; i++)
			result[i] = 0.0;

		//Creating a window that will reverse the effects of the Hann window.
		//If you do not reverse the windowing, it will lead to to unwanted noise.
		float window[FRAMESIZE + 1];
		stft::hanning(FRAMESIZE, window);
		for (int i = 0; i <= FRAMESIZE; i++)
			window[i] *= (2.0f / 3.0f);

		//Allocating memory
		fftw_complex* in = (fftw_complex *) fftw_malloc(
				sizeof(fftw_complex) * (FRAMESIZE / 2 + 1));
		double* out = (double *) fftw_malloc(
				sizeof(double) * (2 * (FRAMESIZE / 2 + 1)));

		//Creating a inversed FFT execution plan
		fftw_plan plan = fftw_plan_dft_c2r_1d(FRAMESIZE, in, out,
				FFTW_ESTIMATE);

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
			for (int k = 0; k < (2 * (FRAMESIZE / 2 + 1)); k++) {
				if (i * OVERLAP + k < frames)
					result[i * OVERLAP + k] += (out[k] * window[k]);
			}
		}
		return result;
	}
};

#endif /* ISTFT_H_ */
