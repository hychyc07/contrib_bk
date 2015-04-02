#ifndef	WALKINGPLANNER__H
#define WALKINGPLANNER__H

#include <yarp/os/all.h>
#include <stdio.h>

using namespace yarp::os;
using namespace std;

#include "footholds.h"	//Here I include the footholds class that will be used to compute robot footholds.
#include "zmpTrajectory.h"
#include "feetTrajectory.h"
/* This class gets a 3d goal position in world reference coordinates ahead of the robot (since initially it will walk only straight) and generates the number of footholds necessary to achieve the goal, along with the corresponding feet and zmp trajectories.*/

class walkingPlanner: public yarp::os::RFModule{

private:
public:
//constructor
walkingPlanner(void); 
//Destructor
~walkingPlanner(void);

virtual bool configure(yarp::os::ResourceFinder &rf);

virtual bool close();
virtual bool getPeriod(void);

virtual bool updateModule(void);
virtual bool interruptModule();

private:
	//Functions
	void computeFootholds(void);
	void computeZmpTrajectory(void);
	void computeFeetTrajectory(void);
};

#endif 