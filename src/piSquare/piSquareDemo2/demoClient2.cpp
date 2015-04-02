// system includes
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

//local includes
#include "demoClient2.h"

using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

Module::Module(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal) : 
		PolicyImprovementManager(rf, movementDuration, start, goal),
		mass(1.0),
		damp(1.0),
		kp(1.0),
		kd(2*sqrt(kp)),
		iteration(0)
{
}

Module::~Module()
{
}

void Module::executeInputTrajectory(dmp::Trajectory& inputTrajectory)
{
	//uses the reference trajectory, provided by the dmp, to control the system
	double y, yd, ydd;
	Vector qTemp, qdTemp, qddTemp;
	Vector u;

	/**
	* initialization of the system trajectories matrices (position, velocity and acceleration)
	* exploiting the parameters held by the base class.
	*/
	q = zeros(numTimeSteps_, numDimensions_);
	qd = zeros(numTimeSteps_, numDimensions_);
	qdd = zeros(numTimeSteps_, numDimensions_);

	/*
	std::stringstream sstm;
	sstm << "data/referenceTrajectory" << iteration << ".txt";
	inputTrajectory.writeTrajectoryToFile(sstm.str());
	*/

	/**
	* vectors holding the system trajectories values for the current time step.
	* Used to automatically keep the values of the previous time step.
	*/
	qTemp = zeros(numDimensions_);
	qdTemp = zeros(numDimensions_);
	qddTemp = zeros(numDimensions_);

	//control vector for the current time step
	u = zeros(numDimensions_);

	for(int i = 0; i < inputTrajectory.getLength(); i++)
	{
		for(int n = 0; n < numDimensions_; n++)
		{
			//get the reference value for the current time step and dimension
			y = inputTrajectory.getTrajectoryPosition(i, n);
			yd = inputTrajectory.getTrajectoryVelocity(i, n);
			ydd = inputTrajectory.getTrajectoryAcceleration(i, n);

			//compute the state for the current time step and dimension
			u(n) = mass * ydd + damp * qdTemp(n) + kp * (y - qTemp(n)) + kd * (yd - qdTemp(n));
			qddTemp(n) = (u(n) - qdTemp(n) * damp) / mass;
			qdTemp(n) = qddTemp(n) * samplingFrequency_ + qdTemp(n);
			qTemp(n) = qdTemp(n) * samplingFrequency_ + qTemp(n);

			//store the state
			q(i, n) = qTemp(n);
			qd(i, n) = qdTemp(n);
			qdd(i, n) = qddTemp(n);
		}
	}

	/*
	std::stringstream sstm2;
	sstm2 << "data/stateTrajectory" << iteration/(numRollouts_+1) << ".txt";
	writeTrajectoryToFile(sstm2.str());
	*/

	return;
}

bool Module::computeCost(dmp::Trajectory& inputTrajectory, Vector& costs)
{
	//firstly uses the input trajectory (i.e. the dmp trajectory) as a control to the actual system
	executeInputTrajectory(inputTrajectory);
	
	//assigns a cost to a given trajectory depending on a certain cost functional
	costs = zeros(numTimeSteps_);
    for (int i = 0; i < numTimeSteps_; ++i)
    {
        for (int d = 0; d < numDimensions_; ++d)
        {
			double c = qdd(i, d);
            c *= c;
            costs(i) += c;
        }
    }

	iteration++;

    return true;
}

bool Module::writeTrajectoryToFile(std::string fileName)
{
	FILE *fp;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", fileName.c_str());
        return false;
    }

	for (int i = 0; i < q.rows(); i++)
    {
		for (int n = 0; n < q.cols(); n++)
        {
			fprintf(fp, "%f\t%f\t%f\t", q(i, n), qd(i, n), qdd(i, n));
        }

		fprintf(fp, "\n");
    }

	fclose(fp);

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
	Vector startPositions = zeros(numDimensions);
    Vector goalPositions = ones(numDimensions);

	Module pi2_test(&rf, movementDuration, startPositions, goalPositions);
	pi2_test.initialize();
	pi2_test.runPI2();
}

