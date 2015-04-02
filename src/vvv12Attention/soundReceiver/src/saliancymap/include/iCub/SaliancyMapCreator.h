/*
 * SaliancyMapCreator.h
 *
 *  Created on: Jul 26, 2012
 *      Author: cdondrup
 */

#ifndef SALIANCYMAPCREATOR_H_
#define SALIANCYMAPCREATOR_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

class SaliancyMapCreator {
private:
	int max_dist;
	ImageOf<PixelMono>* glob_map;
public:
	ImageOf<PixelMono> createMap(int direction, int volume, int distance);
	void freeMap();
};




#endif /* SALIANCYMAPCREATOR_H_ */
