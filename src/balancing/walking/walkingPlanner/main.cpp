// Main Walking Planner module
/* This module instantiates an object walkingPlanner of class walkingPlanner which in turn uses classes footholds, zmpTrajectory, feetTrajectory to compute footholds and walking trajectories for the zmp and feet*/

#include <stdio.h>
#include <yarp/os/all.h>
using namespace yarp::os;

#include "walkingPlanner.h"

int main(int argc, char*argv[])
{
	Network yarp;

	walkingPlanner walkingPlanner;
	
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.configure("ICUB_ROOT", argc, argv);

	return walkingPlanner.runModule(rf);
}