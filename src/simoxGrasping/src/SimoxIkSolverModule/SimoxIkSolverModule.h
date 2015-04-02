#ifndef __Simox_IkSolver_Module_h__
#define __Simox_IkSolver_Module_h__


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
#include <VirtualRobot/IK/PoseQualityManipulability.h>

#ifdef EXTENDED_MANIPULABILITY_MEASURE
//#include "/usr/local/src/robot/niko/ManipulabilityAnalysis/Manipulability.h"
//#include "/usr/local/src/robot/niko/IKQuality/PoseQualityExtendedManipulability.h"
#include "VirtualRobot/Workspace/Manipulability.h"
#include "VirtualRobot/IK/PoseQualityExtendedManipulability.h"
#endif

#include "../SimoxRobotViewer/SimoxRobotViewer.h"

/*!
	This module can be used to search IK solutions, either with iCub's Cartesian Interface or with a Jacobian based generic IK
	Solver that is part of Simox. 
	It can be used to efficiently filter a set of precomputed grasps for an object at a specific location w.r.t. reachability information.
	
*/
class SimoxIkSolverModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum IkMethod
	{
		SimoxGenericIkSolver,		// Simox generic ik solver 
		iCubCartesianIkSolver		// query iCub ik solver
	};

	SimoxIkSolverModule();

    double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule();

    /*
    * Message handler. Just echo all received messages.
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

	VirtualRobot::GraspPtr searchIK(VirtualRobot::ManipulationObjectPtr object);
	VirtualRobot::GraspPtr searchIK(const std::string &objectName);

	/*!
		Add an object to environment. The file must be a valid VirtualRobot::ManipulationObject file.
		To remove an object call setObject(name,"");
		\return True if object is present after returning from this method, false if not.
	*/
	bool setObject( const std::string &objectName, std::string filename );

	Eigen::Matrix4f getObjectPose( const std::string &objectName );
	bool setObjectPose( const std::string &objectName, Eigen::Matrix4f &pose );

	/*!
		Filter all grasps for the object according to their reachability. Only reachable grasps (for the current object pose)
		are returned.
	*/
	VirtualRobot::GraspSetPtr getReachableGrasps(const std::string &objectName);

	/*!
		Returns the manipulability of all reachable graps.
	*/
	bool getManipulabilityGrasps(const std::string &objectName, std::vector<float> &manip,	std::vector<VirtualRobot::GraspPtr> &grasps);

	bool hasReachability();
	bool hasManipulability();
protected:
	bool loadRobot( const std::string &filename );
	bool loadReachability( const std::string &filename );
	bool loadManipulability( const std::string &filename );
	bool selectRNS(const std::string &rns);
	bool selectEEF(const std::string &eef);
	bool selectEEF(const std::string &eef, const std::string &eefPreshape);
	bool setupIkSolver();
	bool setupViewer();
	VirtualRobot::GraspPtr searchIKSimox(VirtualRobot::ManipulationObjectPtr object);
	VirtualRobot::GraspPtr searchIKiCubModule(VirtualRobot::ManipulationObjectPtr object);
	VirtualRobot::GraspPtr getGrasp( const std::string &objectName, const std::string &graspName );
	bool searchGraspIKiCubModule( VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g, std::vector<float> &storeJointValues );
	bool searchIKGrasp(const std::string &objectName, const std::string &graspName);
	bool searchIKGrasp(VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g);
	bool searchGraspIKSimox(VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g, std::vector<float> &storeJointValues);
	void updateViewer( const std::string &objectName );

	float getManipulability();
	VirtualRobot::RobotPtr robot;
	VirtualRobot::EndEffectorPtr currentEEF;
	VirtualRobot::ReachabilityPtr reachSpace;
	VirtualRobot::RobotNodeSetPtr currentRNS;

	VirtualRobot::RobotNodeSetPtr rnsCD_ikChain;
	VirtualRobot::RobotNodeSetPtr rnsCD_robot;

	bool cdEnabled;
	bool cdEnvironment;

	IkMethod ikMethod;

	std::vector<float> ikInitConfig;

	VirtualRobot::GenericIKSolverPtr ikSolver;

	bool isLeft;

	std::map< std::string, VirtualRobot::ManipulationObjectPtr > objects;

	SimoxRobotViewerPtr viewer;
	VirtualRobot::CDManagerPtr cdm;

	std::string moduleName;
	std::string robotBase;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages

	// ikSolveriCub
	yarp::dev::PolyDriver         iCubIkSolverClient;
	yarp::dev::ICartesianControl *iCubIkSolverControl;


#ifdef EXTENDED_MANIPULABILITY_MEASURE
	VirtualRobot::ManipulabilityPtr manipulability;
	VirtualRobot::PoseQualityExtendedManipulabilityPtr qualityMeasure;
#else
	VirtualRobot::PoseQualityManipulabilityPtr qualityMeasure;
#endif
};


typedef boost::shared_ptr<SimoxIkSolverModule> SimoxIkSolverModulePtr;

#endif
