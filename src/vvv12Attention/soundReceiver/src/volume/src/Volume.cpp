/*
 * Volume.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: cdondrup
 */

#include "iCub/Volume.h"

#define MAX(X,Y) X>Y?X:Y

using namespace std;
using namespace yarp::sig;

int Volume::getLoudestChannel(Sound sound) {
	int max[2] = { 0, 0 };
	for (int i = 0; i < sound.getSamples(); i++) {
		max[0] =
		MAX(max[0],(int)abs((double)sound.getSafe(i,0)));
		max[1] =
		MAX(max[1],(int)abs((double)sound.getSafe(i,1)));
	}
	distance = abs((double) (max[0] - max[1]));
	if (abs((double) (max[0] - max[1])) > threshold)
		return max[0] > max[1] ? 0 : 1;
	return -1;
}

int Volume::getVolume(Sound sound, int channel) {
	int result = 0.0;
	for (int i = 0; i < sound.getSamples(); i++) {
		result += (int) abs((double) sound.getSafe(i, channel));
	}
	return result / sound.getSamples();
}

