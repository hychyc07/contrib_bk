

#ifndef MODULE_H_
#define MODULE_H_

//yarp includes
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

//local includes
#include <iCub/piSquare/policyImprovementManager/policyImprovementManager.h>

using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;


class Module : public PolicyImprovementManager
{

public:

	/**
	* class constructor
	* @param
	* @param
	* @param
	* @param
	*/
    Module(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal);

	/**
	* class destructor
	*/
    virtual ~Module();

private:

	///second order system state matrices (timeSteps x dimensions)
	yarp::sig::Matrix q;
	yarp::sig::Matrix qd;
	yarp::sig::Matrix qdd;

	///point mass coefficients
	double mass;
	double damp;

	///PD controller constants
	double kp;
	double kd;

	///keeps track of the current iteration index
	int iteration;

	/**
	* 
	* @param
	*/
	void executeInputTrajectory(dmp::Trajectory& inputTrajectory);

	/**
	* 
	* @param
	* return true if the operation is successful, false otherwise
	*/
    bool computeCost(dmp::Trajectory&, Vector&);

	/** 
	* stores a trajectory as a .txt file
	* @param fileName is the file path
	* @return true if operation is successful, false otherwise
	*/
	bool writeTrajectoryToFile(std::string fileName);

};


#endif /* MODULE_H_ */
