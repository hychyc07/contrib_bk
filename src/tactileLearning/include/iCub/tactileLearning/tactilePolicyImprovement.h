#ifndef __ICUB_TACTILE_POLICY_IMPROVEMENT_H__
#define __ICUB_TACTILE_POLICY_IMPROVEMENT_H__

//yarp includes
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>

//local includes
#include <iCub/piSquare/policyImprovementManager/policyImprovementManager.h>
#include <iCub/tactileLearning/robotInterface.h>

using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub{

namespace tactileLearning{

class TactilePolicyImprovement : public PolicyImprovementManager
{

public:

	/**
	* class constructor
	* @param rf is a resource finder to manage input parameters from configuration file
	* @param movementDuration is the time duration of the physical movement to be optimized
	* @param start is the start position vector for the movement (an element for each dimension)
	* @param goal is the goal position vector for the movement (an element for each dimension)
	* @param averagedCost is a flag indicating if the cost related to the current noise-less trial has to be averaged
	* over a certain number of identical trials or not
	*/
    TactilePolicyImprovement(ResourceFinder* rf, RobotInterface* ri, double movementDuration, Vector& start, Vector& goal, bool averagedCost = false);

	/**
	* class constructor
	* @param rf is a resource finder to manage input parameters from configuration file
	* @param movementDuration is the time duration of the physical movement to be optimized
	* @param dim is the number of dimensions
	* @param initialTrajectory is the initial trajectory used for exploration by PI^2 algorithm
	* @param averagedCost is a flag indicating if the cost related to the current noise-less trial has to be averaged
	* over a certain number of identical trials or not
	*/
	TactilePolicyImprovement(ResourceFinder* rf, RobotInterface* ri, double movementDuration, int dim, dmp::Trajectory& initialTrajectory, bool averagedCost = false);

	/**
	* class destructor
	*/
    virtual ~TactilePolicyImprovement();

private:

	/**
	* assigns a cost to a given trajectory once it was executed and state was acquired
	* @param trajectory is the output trajectory of the DMPs to be executed
	* @param cost is the cost assigned to the input trajectory
	* return true if the operation is successful, false otherwise
	*/
    bool computeCost(dmp::Trajectory& trajectory, Vector& cost);

	/**
	* evaluates the state values obtained during the execution of the trajectory
	* depending on a certain cost functional
	* @param costs is a vector whose elements correspond to the cost assigned for a certain time step
	* @param trajectory is the trajectory being evaluated
	* return true if the operation is successful, false otherwise
	*/
    bool evaluateState(dmp::Trajectory& trajectory, Vector& costs);

	/**
	* eavaluate the state according to the definition of task 2 (i.e. "learn the sufficient pressure to grasp")
	* @param costs is a vector whose elements correspond to the cost assigned for a certain time step
	* @param trajectory is the trajectory being evaluated
	* return true if the operation is successful, false otherwise
	*/
    bool task2(dmp::Trajectory& trajectory, Vector& costs);

	/** 
	* stores a trajectory as a .txt file
	* @param fileName is the file path
	* @return true if operation is successful, false otherwise
	*/
	bool writeTrajectoryToFile(std::string fileName);

	/** 
	* defines a metric for evaluating the difference between a current pressure value and the desired one. 
	* @param pressure is the current pressure value to be compared with the desired one
	* @param timeStep is the current time step, used to introduce a time dependency for the error
	* @param finger is used to specify which finger is considered during error computation
	* return the pressure error for the current time step
	*/
	double computePressureError(double pressure, int timeStep, int finger);

	/** 
	* constructs a matrix containing the desired pressure values, for each finger and each time step
	*/
	void buildDesiredPressures();

	/// state of the tactile sensors [time steps] x [dimensions]
	yarp::sig::Matrix pressureState;

	/// state of the position encoders [time steps] x [joints number]
	yarp::sig::Matrix positionState;

	/// pressure desired trajectory
	yarp::sig::Matrix desiredPressures;

	/// provides methods to deal with the robot 
	RobotInterface* robot;

	int iteration;
};

} // namespace tactileLearning

} // namespace iCub

#endif
