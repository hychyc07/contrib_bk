/*
 * SkeletonReader.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef SKELETONREADER_H_
#define SKELETONREADER_H_

#include <yarp/os/all.h>
#include <yarp/os/Stamp.h>

#include <stdio.h>
#include <iostream>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;

//Positions of the desired coordinates in the KinectDeviceLocal Bottle
//Taken from the OpenNI enumerate XN_SKEL_HEAD, ...etc.
#define LEFT_HAND_POS 8
#define RIGHT_HAND_POS 14
#define HEAD_POS 0
#define CHEST_POS 2

/**
 * This class listens on the Skeleton port and creates a new SkeletonData object when there is new data.
 */
class SkeletonReader : public Thread
{
private:

   /* class variables */
	bool stoploop, emulator;
	Bottle* bot;
	Stamp time;

   /* thread parameters: */
   BufferedPort<Bottle> *in;
   DataCollector *parent;

public:

   /* class methods */

   /**
   * Creates a new SkeletonReader object with the given port and a data collector instance
   * which will collect the gathered data.
   * @param in The BufferdPort to read the skeleton data from
   * @param parent The DataCollector instance which will collect the gathered data
   * @param emulator Starts the emulator mode.
   */
   SkeletonReader(BufferedPort<Bottle> *in, DataCollector* parent, bool emulator);
   bool threadInit();
   void threadRelease();

   /**
   * Reads from the Port instance given on creation.
   * And sends the gathered data to the DataCollector.
   */
   void run();

   /**
    *  Sets a bool to end the loop in the run function.
    */
   void stopLoop()
	{
	   stoploop = true;
	}
};

#endif /* SKELETONREADER_H_ */
