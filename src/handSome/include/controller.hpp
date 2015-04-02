//Generic controller class. The method run() is reimplemented in the children classes;
//it will contain the controller specific code.
//The vector setPoints contains the setpoints in joint space (torques for a joint force 
//controller and positions for a joint position/impedance controller).

//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <iostream>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace HandSome
{
	class ControllerThd: public RateThread
	{
	 
	 protected:
	 	Vector setPoints;			//Vector of the current setpoints
	 	Semaphore* jointMutex;		//Mutex on the setpoints vector in joint space
	 	Semaphore* cartMutex;		//Mutex on the setpoints vector in cartesian space
	 	//Robot robot;				//Object representing the robot to control
	 	
	 public:
	 	ControllerThd(const double);
	 	
	 	//Method called remotely to define the setpoints of the controller
	 	bool setJoints(Vector&);
	 	
	 	virtual bool threadInit() = 0;
	 	virtual void run() = 0;
	 	virtual void threadRelease() = 0;
	 	 
	};
}
#endif /* CONTROLLER_HPP_ */
