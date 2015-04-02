
#ifndef TASKTHREAD_H
#define TASKTHREAD_H


//#include <cstdlib>
//#include <iostream>
//#include <qvector.h>
//#include <vector>
#include <string>
#include <fstream>
// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>

#include <iostream>

#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class TaskThread : public RateThread
{
public:
   TaskThread(int period);
  //virtual ~TaskThread();
   virtual void setNextTaskPosition(Vector pos) = 0;
   virtual bool runNextTask( ) = 0;
   virtual bool reset() = 0;
   virtual bool terminateCurrentTask( ) = 0;
   
   Vector nextPos;
   bool taskDone;
   bool resetDone;
   double restTime;
	
};
#endif 

