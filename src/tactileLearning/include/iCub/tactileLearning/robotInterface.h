#ifndef __ICUB_ROBOT_INTERFACE_H__
#define __ICUB_ROBOT_INTERFACE_H__

// system includes
#include <iostream>
#include <string>

// yarp includes
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h> 
#include <yarp/dev/ControlBoardInterfaces.h>

// local includes
#include <iCub/piSquare/policyLibrary/dmpPolicy.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace library;

namespace iCub{

namespace tactileLearning{

class RobotInterface
{

public:

	/**
    * Class constructor: it acquires already initialized objects for the interaction with the robot.
    * @param robotDev provides methods for controlling the robot
	* @param pos is an interface for position control
	* @param vel is an interface for velocity control
	* @param tactileDataPort is a buffered port for acquiring pressure data from tactile sensors
	* @param activeFingers is a vector representing which fingers are involved in the computation
	* @param activeJoints is a vector representing which joints are involved in the computation; the
	* dimension is equal to the number of joints of the whole arm.
    */
	RobotInterface(PolyDriver* robotDev, IPositionControl* pos, IVelocityControl* vel, IEncoders* enc, BufferedPort<Vector>* tactileDataPort,
				   std::vector<bool>& activeFingers, std::vector<bool>& activeJoints);
	
	/**
	* Class destructor
	*/
	~RobotInterface();

	/**
    * release robot devices and return to home position
    */
	void robotRelease();

	/**
	* executes a trajectory (position/velocity) on the robot by using the input trajectory as a reference trajectory
	* @param inputTrajectory is the reference trajectory, which can be a position as well as a velocity trajectory
	* @param dt is the discretization time step which determines for how much time a single
	* reference value has to be tracked.
	* @param velocityControl is true if a velocity control is requested, false if a position control is requested
	* @return true if operation is successful, false otherwise
	*/
	bool performTrajectory(dmp::Trajectory& inputTrajectory, double dt, bool velocityControl);

	/**
    * increases joints position (with constant velocity) until a pression for each finger is detected
    * @param startPosition is a vector whose elements are the initial positions of each joint
	* @param endPosition is a vector whose elements are the final positions of each joint
	* @return true if operation is successful, false otherwise
    */
	bool moveHandUntilTouch(Vector& startPosition, Vector& endPosition);

	/**
	* performs position control of the arm by keeping fixed the joints of the hand. During the trajectory,
	* the perceived pressure is stored.
	* @param targetPosition is a vector containing the final position requrested for each arm joint
	* @param pressure is a vector of vectors, each one containing the perceived pressure at the give time instant
	* @param dt is the time step between a pressure check and the following one
	* @return true if operation is successful, false otherwise
	*/
	bool positionMoveArm(Vector& targetPosition, std::vector<Vector>& pressure, double dt);

	/**
    * provides the home position vector
    * @param home is the output vector where the home positions are written
	* @return true if operation is successful, false otherwise
    */
	bool getHomePosition(Vector& home);

	/**
    * provides the pressure state matrix
    * @param pressureState is the output pressure state matrix
    */
	void getPressureState(yarp::sig::Matrix& pressureState);

	/**
    * provides the position state matrix
    * @param positionState is the output position state matrix
    */
	void getPositionState(yarp::sig::Matrix& positionState);

	/**
    * provides the velocity state matrix
    * @param velocityState is the output velocity state matrix
    */
	void getVelocityState(yarp::sig::Matrix& velocityState);

	/**
    * gets the number of active fingers, i.e. those actually used during reinforcement learning
	* @returns the number of active fingers
    */
	int getActiveFingers();

	/**
    * gets the number of joints
	* @returns the number of joints
    */
	int getJointsNumber();

	/**
    * reads the current pressure for each active finger
	* @param pressure is the vector where current pressures are written
    */
	void getCurrentPressure(Vector& pressure);

	/**
    * stores the performed trajectories in three different files (position, velocity and pressure)
	* @return true if operation is successful, false otherwise
    */
	bool writeTrajectoriesToFiles();

private:

	/**
    * performs position control, used to set hand joints to an home configuration
    * @param targetPosition is the joints configuration to be reached
	* @return true if operation is successful, false otherwise
    */
	bool positionMoveHand(Vector& targetPosition);

	/**
    * performs an input trajectory by following each reference value for timeStep seconds.
	* After each step, the current pressure, position and velocity state values are acquired.
    * @param targetTrajectory is the reference velocity
	* @param timeStep is the time in which each reference velocity element of the trajectory has to be mantained
	* @param velocityControl is true if a velocity control is requested, false if a position control is requested
	* @return true if operation is successful, false otherwise
    */
	bool controlHand(dmp::Trajectory& targetTrajectory, double timeStep, bool velocityControl);

	/**
    * constructs a reference matrix whose rows are the reference commands to be given at each time step, taking into
	* account only the active joints.
    * @param targetTrajectory is the reference trajectory
	* @param velocityControl is true if a velocity control is requested, false if a position control is requested
	* @param dt is the time step used for the control
    */
	void buildRefTrajectoryMatrix(dmp::Trajectory& targetTrajectory, bool velocityControl, double dt);

	/**
    * reads the current pressure state from tactile sensors (it should be implemented as a sequence of calls to getCurrentPressure)
	* @param step is the current time step of the algorithm
	* @return true if operation is successful, false otherwise
    */
	bool acquirePressureState(int step);

	/**
    * reads the current pressure state from tactile sensors and determines if the object has been touched or not
	* @param velocityCommand is a vector containing the velocity commands for the fingers. It is updated depending
	* on which are the finger which still have to touch the object 
	* @return true if operation is successful, false otherwise
    */
	bool checkIfTouched(Vector& velocityCommand);

	/**
    * scans the position encoders matrix filled during the motion and extracts the columns corresponding to the active joints (noise avoiding)
	* @return true if operation is successful, false otherwise
    */
	bool acquireTrajectoryState();

	/**
    * contact the skinManager module via rpc to ask for a calibration. This is required because the value of the perceived pressure
	* becomes lower and lower at every almost IDENTICAL trajectory.
    */
	void askForCalibration();

	/// driver for controlling the robot
	PolyDriver* robotDevice;

	/// for position control
	IPositionControl *pos;

	/// for velocity control
	IVelocityControl *vel;

	/// encoders values
	IEncoders *encs;

	/// port used to read sensors data
	BufferedPort<Vector>* tactileDataPort;

	/// home position to come back after each trajectory generation
	Vector homePosition;

	/// reference trajectory matrix [time steps] x [joints number]
	yarp::sig::Matrix refTrajectory;

	/// reference position trajectory matrix [time steps] x [joints number] used only for velocity control
	yarp::sig::Matrix refPosTrajectory;

	/// actual position trajectory matrix [time steps] x [joints number]
	yarp::sig::Matrix positionTrajectory;

	/// actual velocity trajectory matrix [time steps] x [joints number]
	yarp::sig::Matrix velocityTrajectory;

	/// pressure state trajectory [time steps] x [active fingers number]
	yarp::sig::Matrix pressureState;

	/// position state trajectory [time steps] x [active joints number]
	yarp::sig::Matrix positionState;

	/// velocity state trajectory [time steps] x [active joints number]
	yarp::sig::Matrix velocityState;

	/// active fingers vector [fingers number]
	std::vector<bool> activeFingers;

	/// active joints vector [joints number]
	std::vector<bool> activeJoints;

	/// number of joints (redundant)
	int jointsNumber;

	/// number of active fingers, computed once instead of retrieving it each time from activeFingers vector
	int activeFingersNumber;

	/// number of active joints, computed once instead of retrieving it each time from activeFingers vector
	int activeJointsNumber;

	static const double POSITION_GAIN;
	static const double POSITION_GAIN_INST;

	/// iteration index (debug)
	int iteration;

	/// rpc port to ask SkinManager for a calibration
	RpcClient rpcSkin;

	/// default name for rpc client port
	static const string RPC_CLIENT_PORT;

	/// default name for rpc server port (i.e. skinManager)
	static const string RPC_SERVER_PORT;
};

}// namespace tactileLearning

}// namespace iCub

#endif

