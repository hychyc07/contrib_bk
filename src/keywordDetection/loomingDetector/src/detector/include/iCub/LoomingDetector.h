/*
 * LoomingDetector.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef LOOMINGDETECTOR_H_
#define LOOMINGDETECTOR_H_

#include <math.h>

#include <yarp/os/all.h>

#include "iCub/Callback.h"
#include "iCub/LoomingWriter.h"

using namespace yarp::os;

/**
 * This is the main calculating thread which will be call when new data arrives and calculate
 * whether there is looming or not.
 */
class LoomingDetector: public Thread, public Callback {
private:

	bool stoploop;
	Semaphore mutex;
	LoomingWriter* writer;
	double mean_left, mean_right;
	double var_left, var_right;
	unsigned long long n_looming_r, nr;
	unsigned long long n_looming_l, nl;
	double dist_sum_right, dist_squared_sum_right;
	double dist_sum_left, dist_squared_sum_left;
	bool loo_r, loo_l;
	double emulator;
	int habituationThreshold;

	/**
	 * This function calculates a mean distance value for each hand and the according variance.
	 * If the current hand distance is closer to the kinect than variance * 2, lomming will be
	 * assumed.
	 * @param dist_hand The current hand distance
	 * @param right true if the dist_hand value is the distance of the right hand, flase if left
	 * @param data A Bottle containing the time stamp and xyz coordinates of the hand
	 */
	bool detect(double dist_hand, bool right, Bottle data);

	/**
	 * This function will be called if the program is started in emulator mode.
	 * It simply takes a fixed distance value and assumes looming if the hand
	 * is closer to the kinect than the specified distance.
	 * @param dist_hand The current hand distance
	 * @param right true if the dist_hand value is the distance of the right hand, flase if left
	 * @param data A Bottle containing the time stamp and xyz coordinates of the hand
	 * @param emulator The fixed distance value
	 */
	bool detectEmulator(double dist_hand, bool right, Bottle data, double emulator);

public:

	/**
	 * Create a new LoomingDetector object containing a LoomingWriter object to post some results,
	 * setting the used semaphore to 1 and initializing all variables with 0 or false.
	 * @param writer The LoomingWriter object which writes to the output port
	 * @param emulator If started with in emulator mode this has to be the desired distance.
	 */
	LoomingDetector(LoomingWriter *writer, double emulator, int habituationThreshold)
		:mutex(1),writer(writer),
		 n_looming_r(0), nr(0),
		 n_looming_l(0), nl(0),
		 mean_left(0), mean_right(0),
		 var_left(0), var_right(0),
		 dist_sum_right(0.0), dist_squared_sum_right(0.0),
		 dist_sum_left(0.0), dist_squared_sum_left(0.0),
		 loo_l(false), loo_r(false),
		 emulator(emulator), habituationThreshold(habituationThreshold) {}
	void receivedSkeletonAndObject(LDData* data);
	void receivedSkeleton(LDData* data, bool object = false);
	void receivedObject(LDData* data);
	void receivedAnyThing();
	bool threadInit();
	void threadRelease();
	void run();

	/**
	*  Sets a bool to end the loop in the run function.
	*/
	void stopLoop()
	{
	   stoploop = true;
	}
};




#endif /* LOOMINGDETECTOR_H_ */
