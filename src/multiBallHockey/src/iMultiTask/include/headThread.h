#ifndef HEADTHREAD_H
#define HEADTHREAD_H

#include "taskThread.h"
#include <yarp/dev/GazeControl.h>

class HeadThread : public TaskThread
{
public:
   PolyDriver         clientGaze;
   IGazeControl  *igaze;
    HeadThread(int period);
   /** main() function of the thread. */
   void run();
   /** Signal the thread to exit the next time it is woken up. */
   void stop();
   bool threadInit();
   void threadRelease();


   
   yarp::sig::Vector homeworld;

	void setNextTaskPosition(Vector pos);
	bool runNextTask( );
	bool reset();
	bool terminateCurrentTask( );
	int startup_context_id;

};

#endif
