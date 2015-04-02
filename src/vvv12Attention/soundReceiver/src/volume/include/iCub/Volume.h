/*
 * Volume.h
 *
 *  Created on: Jul 25, 2012
 *      Author: cdondrup
 */

#ifndef VOLUME_H_
#define VOLUME_H_

#include <iostream>
#include <cmath>
#include <yarp/sig/Sound.h>

using namespace std;
using namespace yarp::sig;

class Volume {
private:
	int distance;
	int threshold;
public:
	Volume(int threshold) :
		threshold(threshold) {}
	int getLoudestChannel(Sound sound);
	int getVolume(Sound sound, int channel);

	int getDistance() const {
		return distance;
	}
};




#endif /* VOLUME_H_ */
