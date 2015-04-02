#ifndef __Simox_GraspExecutionModule_h__
#define __Simox_GraspExecutionModule_h__

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/GazeControl.h>

#include <string.h>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <MotionPlanning/CSpace/CSpacePath.h>

#include "../SimoxRobotViewer/SimoxRobotViewer.h"

/*!
	This module can be used to execute trajectories. Therefore either iCub's velocity controller or the Cartesian Interface can be used.
	Currently only Torso+LeftArm and Torso+RigthArm are supportded (10 DoF)
*/
class SimoxGraspExecutionModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	enum ExecutionControl
	{
		eCartControl,		//! use iCub's Cartesian Controller interface to move along the tcp's workspace trajectory
		eVelControl			//! use iCub's velocity control to move the joints directly
	};

	enum ExecutionJoints
	{
		eTorsoLeftArm,
		eTorsoRightArm
	};

	enum ExecutionState
	{
		eNotMoving,
		eMoving
	};
	

	SimoxGraspExecutionModule();

    double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule();

    /*
    * Message handler.
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply); 


    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf);

    /*
    * Interrupt function.
    */
    bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    bool close();

	// print info
	void print();



protected:
	bool loadRobot( const std::string &filename );
	
	bool selectRNS(const std::string &rns);
	bool setupViewer();

	//! Move hand by given delta. Root coords, Meter.
	bool moveEef(float delta_x, float delta_y, float delta_z);

	bool setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max );
	bool selectEEF(const std::string &eef, const std::string &eefPreshape);
	bool selectEEF(const std::string &eef);

	bool startExecuteMotion (VirtualRobot::TrajectoryPtr t);
	bool getCurrentDistToPose(const Eigen::Matrix4f &globalPose, float &storeDistPosMM, float &storeDistOriDeg, bool lockMutex=true);
	bool stopMotion();
	bool executeMotion( );
	bool goToCartesianPose(const Eigen::Matrix4f &globalPose, bool lockMutex=true);
	bool setupCartesianControl();

	bool updateJointValues();
	bool executeMotionCartControl( );
	bool executeMotionVelControl( );

	bool closeHand();
	bool openHand();
	void getGraspOptions(yarp::os::Bottle &b, yarp::sig::Vector &openPoss, yarp::sig::Vector &closePoss, yarp::sig::Vector &vels);
	bool moveHand(yarp::sig::Vector *p);
	bool safeVelocityMoveTorsoArm(Eigen::VectorXf &vel);
	void showCurrentRobotState();
	void lookToTable();
	VirtualRobot::RobotPtr robot;
	VirtualRobot::RobotNodeSetPtr currentRNS;
	VirtualRobot::RobotNodePtr rootNode;
	VirtualRobot::EndEffectorPtr currentEEF;

	SimoxRobotViewerPtr viewer;

	std::string moduleName;
	std::string robotBase;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages	
	    
	ExecutionJoints executionJoints;
	ExecutionState executionState;
	ExecutionControl controlMode;

	VirtualRobot::TrajectoryPtr currentTrajectory;
	std::vector< Eigen::Matrix4f > currentTcpTrajectory;
	int currentTrajectoryPos;
	double timeStampOfLastPathPoint;

	double timeUpdateEncodersS;
	double lastEncoderUpdateS;

	// parameters to control the movement (and speed)
	double maxDelayExecutionSec;
	float distSwitchToNextPoseMM;
	float distSwitchToNextPoseDeg;
	float distGoalReachedMM;
	float distGoalReachedDeg;

	yarp::dev::PolyDriver         iCubCartesianControlClient;
	yarp::dev::ICartesianControl *iCubCartesianControl;

	//! gaze control
	yarp::dev::PolyDriver iCubGazeClient;
	yarp::dev::IGazeControl *iCubGazeControl;

	yarp::sig::Vector openHandConfig,closeHandConfig,handVel;
	yarp::sig::Vector jointValuesArm,jointValuesTorso, jointLimitsTorsoArmMin, jointLimitsTorsoArmMax;
	yarp::dev::PolyDriver robotDeviceArm,robotDeviceTorso;
	yarp::dev::IEncoders *encArm,*encTorso;
	yarp::dev::IVelocityControl *velArm,*velTorso;

	yarp::os::Semaphore mutex;
	bool verbose;

	bool moveHeadLookTable;
};


typedef boost::shared_ptr<SimoxGraspExecutionModule> SimoxGraspExecutionModulePtr;

#endif
