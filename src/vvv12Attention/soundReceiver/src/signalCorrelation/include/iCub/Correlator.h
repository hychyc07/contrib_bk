/*
 * Correlator.h
 *
 *  Created on: Jul 23, 2012
 *      Author: cdondrup
 */

#ifndef CORRELATOR_H_
#define CORRELATOR_H_

#include <cmath>
#include <iostream>

#define SPEED_OF_SOUND 344.0

using namespace std;

class Correlator {
private:

public:
	float* createEnvelope(float* samples, int size, float beta = 0.9995);
	float* createOnsets(float* envelope, int size);
	int crossCorrelate(float** onsets, int size);
	float getAngle(int distance, int sampleRate, float micDistance=0.3);
};




#endif /* CORRELATOR_H_ */
