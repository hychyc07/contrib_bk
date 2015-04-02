/*
 * Correlator.cpp
 *
 *  Created on: Jul 23, 2012
 *      Author: cdondrup
 */

#include "iCub/Correlator.h"

#define MAX(X,Y) X>Y?X:Y
#define D 50

using namespace std;

float* Correlator::createEnvelope(float* samples, int size, float beta) {
	float* result = new float[size];
	result[0] = abs(samples[0]);
	for (int i = 1; i < size; i++) {
		result[i] = MAX(beta*result[i-1],abs(samples[i]));
	}
	return result;
}

float* Correlator::createOnsets(float* envelope, int size) {
	float* result = new float[size];
	result[0] = 0.0;
	for (int i = 1; i < size; i++) {
		result[i] = MAX(0.0,envelope[i]>envelope[i-1]?envelope[i]:0.0);
	}
	return result;
}

int Correlator::crossCorrelate(float** onsets, int size) {
	int result = 0;
	float max = 0.0;
	for (int d = 0; d < D; d++) {
		float sum = 0.0;
		for (int i = D - 1; i < size - D; i++) {
			sum += (onsets[0][i] * onsets[1][i - d]);
		}
		if (sum > max) {
			max = sum;
			result = -d;
		}
		for (int i = D - 1; i < size - D; i++) {
			sum += (onsets[1][i] * onsets[0][i - d]);
		}
		if (sum > max) {
			max = sum;
			result = d;
		}
	}
	return result;
}

float Correlator::getAngle(int distance, int sampleRate, float micDistance) {
//	cout << "Samples: " << (double) distance << "/" << (double) sampleRate
//			<< "=" << ((double) distance / (double) sampleRate) << endl;
//	cout << "Speed: " << SPEED_OF_SOUND<<endl;
//	cout << "Zähler: "
//			<< (SPEED_OF_SOUND * ((double) distance / (double) sampleRate))
//			<< endl;
//	cout << "Bruch: "
//			<< (double) (SPEED_OF_SOUND
//					* ((double) distance / (double) sampleRate)) / micDistance
//			<< endl;
//	cout << "asin: "
//			<< asin(
//					(double) (SPEED_OF_SOUND
//							* ((double) distance / (double) sampleRate))
//							/ micDistance) << endl;
	double equ = (double) (SPEED_OF_SOUND
			* ((double) distance / (double) sampleRate)) / micDistance;
//	cout<<"Equation: "<<equ<<endl;
	return -1.0 < equ && equ < 1.0 ? asin(equ) * 180.0 / M_PI : 0;
}

