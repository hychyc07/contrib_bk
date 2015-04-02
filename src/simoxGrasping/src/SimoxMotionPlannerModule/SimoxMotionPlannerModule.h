#ifndef __Simox_MotionPlannerModule_h__
#define __Simox_MotionPlannerModule_h__

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

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
	This module can be used to search collision free motions with RRT-based algorithms.
	Therefore Simox' sampling based motion planning library Saba is used.
	
*/
class SimoxMotionPlannerModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimoxMotionPlannerModule();

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


	/*!
		Add an object to environment. The file must be a valid VirtualRobot::ManipulationObject file.
		To remove an object call setObject(name,"");
		\return True if object is present after returning from this method, false if not.
	*/
	bool setObject( const std::string &objectName, std::string filename );

	Eigen::Matrix4f getObjectPose( const std::string &objectName );
	bool setObjectPose( const std::string &objectName, Eigen::Matrix4f &pose );
	
	/*!
	    Plan a collision-free motion for the current setup from start to goal.
	*/    
	bool planMotion(const std::vector<float> startConfig, const std::vector<float> goalConfig);

protected:
	bool loadRobot( const std::string &filename );
	
	bool selectRNS(const std::string &rns);
	bool setupViewer();

	bool checkCollisionGoalConfig(const std::vector<float> &config, VirtualRobot::SceneObjectSetPtr colModel, std::map<VirtualRobot::ManipulationObjectPtr, std::vector<float> > &storeOriginalPoses);
	bool restoreCollisionGoalConfig(std::map<VirtualRobot::ManipulationObjectPtr, std::vector<float> > &originalPoses);
	bool checkObjectMove(VirtualRobot::ManipulationObjectPtr o, VirtualRobot::SceneObjectSetPtr colModel, float dx, float dy, float dz );
	bool setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max );
	bool selectEEF(const std::string &eef, const std::string &eefPreshape);
	bool selectEEF(const std::string &eef);
	VirtualRobot::RobotPtr robot;
	VirtualRobot::RobotNodeSetPtr currentRNS;

	VirtualRobot::RobotNodeSetPtr rnsCD_robotMove;
	VirtualRobot::RobotNodeSetPtr rnsCD_robotStatic;

	Saba::CSpacePathPtr currentSolution;
	Saba::CSpacePathPtr currentSolutionOpti;
	int pathOptimizingSteps;

	bool cdEnabled;
	bool cdEnvironment;
	bool searchColFreeObjectPose; // move target around in case an inaccurate ik solution results in a collision

	std::vector<float> ikInitConfig;

	std::map< std::string, VirtualRobot::ManipulationObjectPtr > objects;

	SimoxRobotViewerPtr viewer;
	VirtualRobot::CDManagerPtr cdm;
	VirtualRobot::EndEffectorPtr currentEEF;

	std::string moduleName;
	std::string robotBase;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages

	
};


typedef boost::shared_ptr<SimoxMotionPlannerModule> SimoxMotionPlannerModulePtr;

#endif
