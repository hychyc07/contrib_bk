// local includes
#include <iCub/tactileLearning/tactileLearningThread.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::tactileLearning;

TactileLearningThread::TactileLearningThread(ResourceFinder* rf, string mn, string rn, bool lh, bool fingers[], bool joints[])
{
	this->rf = rf;
	this->moduleName = mn;
	this->robotName = rn;
	this->leftHand = lh;
	this->activeFingers = fingers;
	this->activeJoints = joints;
}

bool TactileLearningThread::threadInit() 
{
	// robot interface initialization: open and connect port to read tactile data
	string skinPortName = "/"+robotName+"/skin";
	if(robotName.compare("icub") == 0)
	{
		if(leftHand) skinPortName.append("/left_hand_comp");
		else skinPortName.append("/right_hand_comp");
	}
	else
	{
		if(leftHand) skinPortName.append("/left_hand");
		else skinPortName.append("/right_hand");
	}
	if (!compensatedTactileDataPort.open( ("/"+moduleName+"/skin:i").c_str() ))
	{
        printf("ERROR: unable to open the tactile data input port\n");
		return false;
	}
	
    if(!Network::connect(skinPortName.c_str(), compensatedTactileDataPort.getName().c_str()) )
	{
        printf("ERROR: unable to connect to tactile data port: %s\n", skinPortName.c_str());
		return false;
    }

	// robot interface initialization: specify parameters for robot interaction
	Property params;
	params.put("robot", robotName.c_str());	
	params.put("device", "remote_controlboard");	
	if(leftHand)
	{
		params.put("part", "left_arm");
		params.put("local", ("/"+moduleName+"/left_arm").c_str());
		params.put("remote", ("/"+robotName+"/left_arm").c_str());		
	}
	else{
		params.put("part", "right_arm");
		params.put("local", ("/"+moduleName+"/right_arm").c_str());
		params.put("remote", ("/"+robotName+"/right_arm").c_str());
	}

	// robot interface initialization: create a device
	PolyDriver* robotDevice = new PolyDriver(params);
	IPositionControl *pos;
	IVelocityControl *vel;
	IEncoders *encs;
	if (!robotDevice->isValid())
	{
		printf("ERROR: device not available.  Here are the known devices:\n");
		printf("%s\n", Drivers::factory().toString().c_str());
		return false;
	}	

	bool ok;
	ok = robotDevice->view(pos);
    ok = ok && robotDevice->view(vel);
	ok = ok && robotDevice->view(encs);
	if (!ok)
	{
		printf("ERROR: problems acquiring interfaces\n");
		return false;
	}

	// robot interface initialization: build mask to distinguish which are the active joints/fingers
	int numJoints;
	pos->getAxes(&numJoints);
	std::vector<bool> jointsMask, fingersMask;
	buildJointsMask(jointsMask, numJoints);
	buildFingersMask(fingersMask);

	robot = new RobotInterface(robotDevice, pos, vel, encs, &compensatedTactileDataPort, fingersMask, jointsMask);

	// PI^2 initialization
	int numActiveJoints = 0;

	for(int i = 0; i < NUM_HAND_JOINTS; i++)
	{
		if(activeJoints[i]) numActiveJoints++;
	}

	Vector startPositions = zeros(numActiveJoints);
	Vector goalPositions = zeros(numActiveJoints);

	// get the start and goal positions for the initial trajectory (PI^2 is local)
	robot->moveHandUntilTouch(startPositions, goalPositions);

	// generate the initial trajectory for PI^2 algorithm
	string parameterName = "num_time_steps";
	if (!rf->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        return false;
    }
	int steps = rf->find(parameterName.c_str()).asInt();

	dmp::Trajectory initialTrajectory(rf);
	generateInitialTrajectory(startPositions, goalPositions, MOVEMENT_DURATION, steps, initialTrajectory);

	piSquare = new TactilePolicyImprovement(rf, robot, MOVEMENT_DURATION, startPositions.size(), initialTrajectory, false);

	//piSquare = new TactilePolicyImprovement(rf, robot, MOVEMENT_DURATION, startPositions, goalPositions);

	return true;
}

void TactileLearningThread::buildJointsMask(vector<bool>& jointsMask, int numJoints)
{
	// set all the arm joints to "inactive"
	for(int i = 0; i < (numJoints - HAND_JOINTS_OFFSET); i++) jointsMask.push_back(false);
	
	// set the hand joints accordingly with the info about the active joints
	for(int i = 0; i < HAND_JOINTS_OFFSET; i++)
	{
		if(activeJoints[i] == true) jointsMask.push_back(true);
		else jointsMask.push_back(false);
	}
}

void TactileLearningThread::buildFingersMask(vector<bool>& fingersMask)
{
	/*
	just copy the elements of activeFingers in fingersMask, but add an
	element in order to consider ring and pinky fingers as separated.
	This is true only in terms of tactile sensors, since each of them
	has its own sensors.
	*/
	for(int i = 0; i < NUM_FINGERS; i++)
	{
		if(activeFingers[i] == true) fingersMask.push_back(true);
		else fingersMask.push_back(false);
	}

	if(activeFingers[NUM_FINGERS-1] == true) fingersMask.push_back(true);
	else fingersMask.push_back(false);
}

bool TactileLearningThread::generateInitialTrajectory(Vector& start, Vector& end, double duration, int steps, dmp::Trajectory& trajectory)
{
	double samplingStep = duration / (double) steps;
    double samplingFrequency = static_cast<double> (1.0) / samplingStep;

	//could be removed
    std::vector<std::string> variableNames;
    for (unsigned int i = 0; i < start.size(); i++)
    {
        std::stringstream ss;
        ss << i;
        variableNames.push_back(std::string("dummy_") + ss.str());
    }

    if (!trajectory.initialize(variableNames, samplingFrequency))
    {
        printf("ERROR: Could not initialize custom trajectory.\n");
        return false;
    }

	// firstly generate a min jerk trajectory from start to goal, whose duration is HOLD seconds shorter (read below)
    if (!dmp::MathHelper::generateMinJerkTrajectory(start, end, MOVEMENT_DURATION-HOLD, samplingStep, trajectory))
    {
		printf("ERROR: Could not generate minimum jerk trajectory.\n");
        return false;
    }

	/*
	trick to ensure that the actual trajectory (i.e. the one which is performed) reaches the target position;
	Such an issue comes from the errors introduced during the position control used to track the trajectory.
	To solve it, the duration of the minimum jerk trajectory is 1 second shorter then the original duration
	given as parameter, and the remaining second is used to keep the final position constant.
	*/
	Eigen::VectorXd lastPoint = Eigen::VectorXd::Zero(start.size() * 3);
	trajectory.getTrajectoryPoint(trajectory.getLength() - 1, lastPoint);
	for(unsigned int i = 0; i < HOLD*(1 / samplingStep); i++)
	{
		if(!trajectory.add(lastPoint))
		{
			printf("ERROR: something was wrong while adding a trajectory point\n");
			return false;
		}
	}
	trajectory.writeTrajectoryToFile("data/customTrajectoryBefore1.txt");
	
	trajectory.computeDerivatives();
	trajectory.writeTrajectoryToFile("data/customTrajectoryBefore2.txt");
	
	//if(!robot->performTrajectory(trajectory, samplingStep, false)) return false;

	//robot->writeTrajectoriesToFiles();

	//yarp::sig::Matrix velocity;
	//robot->getVelocityState(velocity);

	// set each velocity trajectory as a position trajectory (because velocity control is needed)
	for(int i = 0; i < trajectory.getLength(); i++)
	{
		for(unsigned int d = 0; d < start.size(); d++)
		{
			if(!trajectory.update(i, d * 3, trajectory.getTrajectoryVelocity(i, d)))
			{
				printf("ERROR: something was wrong during position/velocity trajectory conversion\n");
				return false;
			}
		}
	}

	// finally compute the derivatives for the new velocity trajectory, i.e. acceleration and jerk
	trajectory.computeDerivatives();

	trajectory.writeTrajectoryToFile("data/customTrajectoryAfter.txt");
	
	/*
	for(int i = 0; i < 10; i++)
	{
	if(!robot->performTrajectory(trajectory, samplingStep, true)) return false;
	}
	*/
	return true;
}

void TactileLearningThread::threadRelease() 
{
	robot->robotRelease();
}

void TactileLearningThread::run()
{
    printf("RUNNING Thread\n\n");

	if(!piSquare->initialize())
	{
		printf("ERROR: impossible to initialize PI^2 algorithm\n");
		return;
	}

	piSquare->runPI2();
}