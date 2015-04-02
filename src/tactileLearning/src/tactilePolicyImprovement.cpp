// system includes
#include <iostream>
#include <fstream>
#include <math.h>

// local includes
#include <iCub/tactileLearning/tactilePolicyImprovement.h>
#include <iCub/tactileLearning/constants.h>

using namespace pi2;
using namespace library;
using namespace iCub::tactileLearning;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


TactilePolicyImprovement::TactilePolicyImprovement(ResourceFinder* rf, RobotInterface* ri, double md, Vector& start, Vector& goal, bool averagedCost) : 
		PolicyImprovementManager(rf, md, start, goal, averagedCost),
		robot(ri),
		iteration(0)
{
	buildDesiredPressures();
}

TactilePolicyImprovement::TactilePolicyImprovement(ResourceFinder* rf, RobotInterface* ri, double md, int d, dmp::Trajectory& trajectory, bool averagedCost) : 
		PolicyImprovementManager(rf, md, d, trajectory, averagedCost),
		robot(ri),
		iteration(0)
{
	buildDesiredPressures();
}

TactilePolicyImprovement::~TactilePolicyImprovement()
{
}

bool TactilePolicyImprovement::computeCost(dmp::Trajectory& inputTrajectory, Vector& costs)
{
	std::stringstream sstm;
	sstm << "data/referenceTrajectory" << iteration << ".txt";
	inputTrajectory.writeTrajectoryToFile(sstm.str());
	iteration++;

	// firstly uses the input trajectory (i.e. the dmp trajectory) as a control to the actual system
	//for(int i = 0; i < 100; i++)
	//{
	if(!robot->performTrajectory(inputTrajectory, samplingFrequency_, true)) return false;
	//}
	robot->getPositionState(positionState);
	robot->getPressureState(pressureState);

	// assign a cost to a given trajectory depending on a certain cost functional
	if(!task2(inputTrajectory, costs)) return false;
	
    return true;
}

bool TactilePolicyImprovement::evaluateState(dmp::Trajectory& trajectory, Vector& costs)
{
	/*
	the cost associated to a trajectory is given by the pressure error penality for each finger, at every time step,
	with respect to a given pressure reference value. The cost on the final time step is the same, but it weights more
	*/
	double totCost = 0;
	costs = zeros(numTimeSteps_);
	double presCost;

	for(int i = 0; i < numTimeSteps_; i++)
	{
		presCost = 0;
	
		// pressure cost
		for(int d = 0; d < pressureState.cols(); d++)
		{	
			double pressureError = computePressureError(pressureState(i, d), i, d);
			if(i < numTimeSteps_ - 10) presCost += pressureError;
			else presCost += pressureError * 1000000;
		}

		totCost += presCost;
		printf("presCost:%e\n\n", presCost);

		costs(i) = presCost;
	}

	printf("\nCOST: %e\n", totCost);

	return true;
}

bool TactilePolicyImprovement::task2(dmp::Trajectory& trajectory, Vector& costs)
{
	/*
	TASK 2 definition:
	-Main task: pressure > 0 for each finger after the object has been picked up
	-Secundary task: reduce the pressure during the grasp

	the final time step is not the real final step of the trajectory any more. Instead, it is the time instant
	after the object has been picked up
	*/

	double totCost = 0;
	costs = zeros(numTimeSteps_);
	double presCost, finalPresCost;
	Vector finalArmPosition;
	Vector finalPressure;
	std::vector<Vector> pressurePickUp;

	// weights for primary and secundary task
	double k1, k2;
	k1 = 0.1;
	k2 = 50;

	// instantaneous pressure cost
	for(int i = 0; i < numTimeSteps_ - 1; i++)
	{
		presCost = 0;

		// pressure cost
		for(int d = 0; d < pressureState.cols(); d++)
		{	
			presCost += pressureState(i, d);
		}

		presCost = k1 * presCost;
		
		totCost += presCost;
		printf("%i-th presCost:%e\n\n", i, presCost);

		costs(i) = presCost;
	}

	// move the arm to the final position
	// NOTE: a dynamic vector is used instead of a yarp matrix because the number of pressures recorded
	// during the movement of the arm is not known a priori
	finalArmPosition = Vector(robot->getJointsNumber() - NUM_HAND_JOINTS, FINAL_POSITION_VECTOR);
	if(!robot->positionMoveArm(finalArmPosition, pressurePickUp, samplingFrequency_/2)) return false;

	// wait a while before checking pressure (the time duration is adjusted so that the total number of
	// pressure checks is always the same
	int initialSize = pressurePickUp.size();
	for(int i = 0; i < numTimeSteps_ - initialSize; i++)
	{
		robot->getCurrentPressure(finalPressure);
		pressurePickUp.push_back(finalPressure);
		Time::delay(samplingFrequency_);
	}

	// final pressure cost
	finalPresCost = 0;
	for (vector<Vector>::iterator it = pressurePickUp.begin(); it != pressurePickUp.end(); ++it)
	{
		for(unsigned int f = 0; f < (*it).length(); f++)
		{
			if((*it)[f] < TOUCH_THRESHOLD)
			{
				finalPresCost += 1;
				//break;
			}
		}
	}

	finalPresCost = k2 * finalPresCost;

	totCost += finalPresCost;
	printf("Final cost: %e\n", finalPresCost);
	costs(numTimeSteps_ - 1) = finalPresCost;

	printf("\nCOST: %e\n", totCost);

	// store pressure to file (both grasp and pick-up)
	std::stringstream sstm;
	FILE *fp;
	string str;

	sstm << "data/pressurePickUp" << iteration-1 << ".txt";
	str = string(sstm.str());
	
	if ((fp = fopen(str.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", str.c_str());
        return false;
    }

	for (int i = 0; i < pressureState.rows(); i++)
    {
		for (int n = 0; n < pressureState.cols(); n++)
        {
			fprintf(fp, "%f\t", pressureState(i, n));
        }

		fprintf(fp, "\n");
    }

	for (vector<Vector>::iterator it = pressurePickUp.begin(); it != pressurePickUp.end(); ++it)
	{
		for(unsigned int f = 0; f < (*it).length(); f++)
		{
			fprintf(fp, "%f\t", (*it)[f]);
		}

		fprintf(fp, "\n");
	}

	fclose(fp);

	return true;
}

double TactilePolicyImprovement::computePressureError(double pressure, int step, int dim)
{
	return pow(pressure - desiredPressures(step, dim), 2);
}

void TactilePolicyImprovement::buildDesiredPressures()
{
	desiredPressures.resize(initialTrajectory_.getLength(), robot->getActiveFingers());
	
	for(int i = 0; i < desiredPressures.rows(); i++)
	{
		for(int d = 0; d < desiredPressures.cols(); d++)
		{
			if(i < desiredPressures.rows() - CONTACT_DURATION) desiredPressures(i, d) = 0;
			//else if(d == 0 || d == 1) desiredPressures(i, d) = 20;
			//else if(d == 2) desiredPressures(i, d) = 10;

			else desiredPressures(i, d) = DESIRED_PRESSURE;
		}
	}
}

bool TactilePolicyImprovement::writeTrajectoryToFile(std::string fileName)
{
	FILE *fp;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", fileName.c_str());
        return false;
    }

	for (int i = 0; i < pressureState.rows(); i++)
    {
		for (int n = 0; n < pressureState.cols(); n++)
        {
			fprintf(fp, "%f\t", pressureState(i, n));
        }

		fprintf(fp, "\n");
    }

	fclose(fp);

	return true;
}

