#include "headThread.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

// constructor

HeadThread::HeadThread(int period)
        : TaskThread(period) 
{	
  
}


bool HeadThread::threadInit(){


	std::cout << "initing head 1.." << std::endl;

   Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!clientGaze.open(optGaze))
            return false;

        // open the view
        clientGaze.view(igaze);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the tracking mode, the neck limits and so on.
        igaze->storeContext(&startup_context_id);

        // set trajectory time:
        igaze->setNeckTrajTime(0.8);
        igaze->setEyesTrajTime(0.4);



 
	//std::cout << "head thread init: goign to home..." << std::endl
	homeworld.resize(3);
	homeworld[0] = -2.0;
	homeworld[1] = 0.0;
	homeworld[2] = -0.12;
	reset();
   return true;		  
}

void HeadThread::threadRelease(){
   // we require an immediate stop
        // before closing the client for safety reason
        igaze->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        igaze->restoreContext(startup_context_id);

}

void HeadThread::stop( )
{

}

void HeadThread::run( )
{
	//std::cout << "head running......" << std::endl;
	runNextTask();
}
// 

void HeadThread::setNextTaskPosition(Vector pos){
	//std::cout << "head: set next pos to" <<  pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
	if ((pos[0] > -3.0) && (pos[0] < -0.5) && (pos[1] > -0.75) && (pos[1] < 0.75) && (pos[2] > -0.2) && (pos[2] < 0.2)){
		nextPos = pos;
		taskDone = false;
		//std::cout << "head: setting next pos to" <<  pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
	}
	else{
		//std::cout << "head: avoiding next pos to" <<  pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
	}
}
bool HeadThread::runNextTask( ){
  if (taskDone) {
	double ttime=Time::now();
	if (ttime - TaskThread::restTime > 1.0){
	  reset();
	  
	}
	return false; 
  }

  bool ret = true;
  bool done = (this)->igaze->lookAtFixationPoint(nextPos);
  
   	    // wait until the operation is done
	  //std::cout << "running next task...:" << nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
// 	  bool doneg=false;
// 	   while (!doneg) {
// 		// temporary: move the head
// 		(this)->igaze->checkMotionDone(&doneg);
// 		Time::delay(0.001);   // or any suitable delay
// 	      }
// 	    taskDone = true;
// 	    resetDone = false;
// 	    restTime=Time::now();
  
  return ret;
}
bool HeadThread::reset(){
   bool ret = true;
	nextPos = ((HeadThread*)this)->homeworld;
  //bool gazeok = (this)->igaze->lookAtFixationPoint(((HeadThread*)this)->homeworld);
	      
// 		   // wait until the operation is done
// 		   bool doneg=false;
// 		   while (!doneg) {
// 		     // temporary: move the head
// 		     (this)->igaze->checkMotionDone(&doneg);
// 		     Time::delay(0.01);   // or any suitable delay
// 		  }

   resetDone = true;
  return ret;
}

bool HeadThread::terminateCurrentTask( ){
  bool ret = false;

  return ret;
}

