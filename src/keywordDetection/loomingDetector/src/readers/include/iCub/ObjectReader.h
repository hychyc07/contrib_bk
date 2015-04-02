/*
 * LDThread.h
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#ifndef OBJECTREADER_H_
#define OBJECTREADER_H_

#include <yarp/os/all.h>

#include <stdio.h>
#include <iostream>

#include "iCub/DataCollector.h"

using namespace std;
using namespace yarp::os;

/**
 * This class listens on the Object port and creates a new ObjectData object when there is new data.
 */
class ObjectReader : public Thread
{
private:

   /* class variables */
	bool stoploop;
	Bottle *bot;

   /* thread parameters: */
   BufferedPort<Bottle> *in;
   DataCollector *parent;

public:

   /* class methods */

   /**
    * Creates a new ObjectReader object with the given port and a data collector instance
    * which will collect the gathered data.
    * @param in The BufferdPort to read the object data from
    * @param parent The DataCollector instance which will collect the gathered data
    */
   ObjectReader(BufferedPort<Bottle> *in, DataCollector* parent);
   bool threadInit();
   void threadRelease();

   /**
    * Reads from the Port instance given on creation.
    * And sends the gathered data to the DataCollector.
    */
   void run();

   /**
    * Sets a bool to end the loop in the run function.
    */
   void stopLoop()
   {
	   stoploop = true;
   }
};

#endif /* OBJECTREADER_H_ */
