//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#include <yarp/os/Time.h>
#include "controller.hpp"

using namespace HandSome;

ControllerThd::ControllerThd(const double period) : RateThread(int(period*1000.0))
{
	jointMutex = new Semaphore();
	cartMutex = new Semaphore();
}

/*
The setpoint vector cannot be read and written at the same time. 
This is controlled by a mutex mechanism.
*/
bool ControllerThd::setJoints(Vector& setPoints)
{
	jointMutex->wait();				//Lock the semaphore
	//cout<<"Writing...start!"<<endl;
	this->setPoints = setPoints;
	//cout<<"Writing...done!"<<endl;
	jointMutex->post();				//Unlock the semaphore
}

