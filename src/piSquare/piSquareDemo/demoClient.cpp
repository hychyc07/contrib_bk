// system includes
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

//local includes
#include "demoClient.h"

using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

Module::Module(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal) : 
		PolicyImprovementManager(rf, movementDuration, start, goal)
{
}

Module::~Module()
{
}

bool Module::computeCost(dmp::Trajectory& state, Vector& costs)
{
	//assigns a cost to a given trajectory depending on a certain cost functional

    costs = zeros(numTimeSteps_);
    for (int i=0; i<numTimeSteps_; ++i)
    {
        for (int d=0; d<numDimensions_; ++d)
        {
			double c = state.getTrajectoryPosition(i, d);
            c *= c;
            costs(i) += c;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
	ResourceFinder rf;
	rf.setVerbose();
	rf.setDefaultConfigFile("piSquareConf.ini");
	rf.setDefaultContext("piSquare/conf");
	rf.configure("ICUB_ROOT", argc, argv);
	
	//parameters
	int numDimensions = 1;
	double movementDuration = 2.0;
	Vector startPositions = ones(numDimensions);
    Vector goalPositions = zeros(numDimensions);

	Module pi2_test(&rf, movementDuration, startPositions, goalPositions);
	pi2_test.initialize();
	pi2_test.runPI2();
}

