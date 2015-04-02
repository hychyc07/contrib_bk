#ifndef __ICUB_TACTILE_LEARNING_THREAD_H__
#define __ICUB_TACTILE_LEARNING_THREAD_H__

// system includes
#include <iostream>
#include <string>
#include <vector>

// yarp includes
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h> 
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>

//local includes
#include <iCub/tactileLearning/tactilePolicyImprovement.h>
#include <iCub/tactileLearning/robotInterface.h>
#include <iCub/tactileLearning/constants.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub{

namespace tactileLearning{

class TactileLearningThread : public Thread
{
public:

	/**
	* Class constructor
	* @param rf is the resource finder used for parameters management
	* @param robotName is the name of the robot used for the application
	* @param moduleName is the name of the module containing the thread
	* @param leftHand indicates if the left hand is used or not
	* @param activeFingers is a vector indicating which fingers are used
	* @param activeJoints is a vector indicating which joints are used
	*/
	TactileLearningThread(ResourceFinder* rf, string moduleName, string robotName, bool leftHand, bool activeFingers[5], bool activeJoints[]);

	/**
	* initialization of both robot interface and PI^2 algorithm
	*/
	bool threadInit();

	/**
	* release of the thread
	*/
	void threadRelease();

	/**
	* actual computation performed by the thread
	*/
	void run();

private:

	/**
	* reduces the scope of the motion only to the requested joints by constructing a vector
	* which says which joints are active. The only difference with respect to the activeJoints
	* vector is the dimension. Here the vector has an element for each joint of the arm.
	* @param jointsMask is a vector indicating if a joint is involved in the motion or not
	* @param numJoints is the number of joints for the robor arm
	*/
	void buildJointsMask(vector<bool>& jointsMask, int numJoints);

	/**
	* reduces the scope of the tactile data acquisition only to the requested fingers by constructing
	* a vector which says which fingers are active. The only difference with respect to the activeFingers
	* vector is the dimension. Here the vector has an element for each finger endowed with tactile sensors,
	* i.e. 5 elements instead of 4.
	* @param fingersMask is a vector indicating if a finger is involved in the computation or not
	*/
	void buildFingersMask(vector<bool>& fingersMask);

	/**
	* generates the initial velocity trajectory for PI^2 algorithm starting from information about initial and
	* goal position. A minimum jerk trajectory is firstly computed, and then only the velocity "component" of
	* such trajectory is considered.
	* @param startPositions is the initial positions vector
	* @param endPositions is the end positions vector
	* @param duration is the time duration of the movement encoded by the trajectory
	* @param steps is the number of time steps used during reinforcement learning
	* @param trajectory is the output (velocity) trajectory to be generated
	*/
	bool generateInitialTrajectory(Vector& startPositions, Vector& endPositions, double duration, int steps, dmp::Trajectory& trajectory);

	/// manages parameters from configuration file
	ResourceFinder* rf;

	///port for retrieving the values of tactile sensors
	BufferedPort<Vector> compensatedTactileDataPort;

	/// the name of the module
	string moduleName;
	
	/// the name of the robot (simulator or real robot)
	string robotName;

	/// indicates which hand is used (left if it is true, right otherwise)
	bool leftHand;

	/// active fingers (i-th finger is used if i-th element is true)
	bool* activeFingers;

	/// active joints (i-th joint is used if i-th element is true)
	bool* activeJoints;

	/// PI^2 class manager
	TactilePolicyImprovement* piSquare;

	/// robot interface providing methods for reading tactile sensors and controlling trajectories
	RobotInterface* robot;
};

} //namespace tactileLearning

} //namespace iCub

#endif