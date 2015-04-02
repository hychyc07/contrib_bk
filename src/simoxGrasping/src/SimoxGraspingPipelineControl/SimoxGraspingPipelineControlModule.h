#ifndef __Simox_Grasping_Pipeline_Control_Module_h__
#define __Simox_Grasping_Pipeline_Control_Module_h__

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

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include "SimoxGraspingPipelineControlWindow.h"

/*!
	This module offers a GUI to control a grasping process. The module connects to
	SimoxIkSolverModule and to SimoxRobotViewerModule.
	The Qt-based GUI window is handled in SimoxGraspingPipelineControlWindow.

	@see SimoxGraspingPipelineControlWindow
	@see SimoxRobotViewerModule
	@see SimoxIkSolverModule
*/
class SimoxGraspingPipelineControlModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimoxGraspingPipelineControlModule();

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

	/*!
		Ads an object to this module. The object is identified via the objectName string.
		The object is registered and visualized in SimoxIkSolverModule and to SimoxRobotViewer.
		@param objectName The name that is used to identify the object later on.
		@param filename	The filename of the ManipulationObject XML file. the filename can be given relatively 
						to a path in VirtualRobot::RuntimeEnvionment::getDataPaths(). 
						@see VirtualRobot::ManipulationObject
		@return True on success, false on failure.

	*/
	bool setObject(const std::string &objectName, std::string filename);

	//! remove the object from SimoxIkSolverModule and SimoxRobotViewer
	bool removeObject( const std::string &objectName);

	//! Set the pose of the object in SimoxIkSolverModule and SimoxRobotViewer (global coord system)
	bool setObjectPose(const std::string &objectName, const Eigen::Matrix4f &pose);

	//! The object's pose is returned (in global coordinate system)
	Eigen::Matrix4f getObjectPose(const std::string &objectName);

	/*! 
		Enable a grasp visualization of object that was registered with name objectName. 
		@param objectName The name to identify the object, @see setObject().
		@graspName The name of the grasp as defined in the ManipulationObject's XML definition.
	*/
	bool showGrasp( const std::string &objectName, const std::string &graspName );

	/*!
		Search an IK solution. For a specific object and grasp.
	*/
	bool searchIK( const std::string &objectName, const std::string &graspName, std::vector<float> &storeJointValues, float& storeManipulabilityQuality );

	/*!
		Get all reachable grasps for the current object position.
	*/
	bool getReachableGrasps(const std::string &objectName, std::vector<std::string> &storeGrasps, std::vector<float> &storeQuality);
	bool showReachableGrasps( const std::string &objectName, std::map< std::string, float> &grasps, bool enable );

	/*!
		Execute the given joint values [rad] for hip and arm.
	*/
	bool goToPoseHipArm(bool left, std::vector<float> jV_rad);
	bool goToPoseHipArm(std::vector<float> jV_rad);

	/*!
		Get current joint values.
		\param jV_rad Values are stored here [rad].
		\param withTorso True: Start with 3 torso joints followed by 7 arm joints. False: only 7 arm joints.
	*/
	bool getCurrentConfigArm(bool left, std::vector<float> &jV_rad, bool withTorso = true);
	bool getCurrentConfigArm(std::vector<float> &jV_rad, bool withTorso = true);

	/*!
		Get the pose offset for the position where the object is located at.
		Stores the result in x,y,z.
		\return True on success.
	*/
	bool queryHandEyeOffset(const std::string &objectName, float &x, float &y, float &z, float &ro, float &pi, float &ya);

	//! set the fixed offset
	//void handEyeOffsetChanged(const std::string& objectName, float x, float y, float z);

	bool planMotion( const std::vector<float> startConfig, const std::vector<float> goalConfig, std::vector< std::vector<float> > &storePath);

	void goToInitPose();

	bool showCurrentRobotState();
	bool showConfiguration(std::vector<float> &config);

	bool showMotion(bool enable, std::vector< std::vector<float> > path);


	bool localizeLego(bool updatePose, Eigen::Matrix4f &m, bool searchForObstacle, Eigen::Matrix4f &obst_pose);

	void setCurrentObject(const std::string &objectName);

	bool checkConnections(bool tryToReconnect);
	bool getConnectionStatus();

	void saveScene(const std::string &filename);

	bool stopMotionExecution(bool stopPathExecution = true, bool stopAllParts=true);
	bool startMotionExecution(std::vector< std::vector<float> > & path);
	
	void setSegThresh(float thresh);

	void openHand();
	void closeHand();

	//! look to table
	void moveHeadStandardPose(float joint0 = -35.0f);

	void liftHand();

protected:
	bool setupCartInterface();
	bool setupConnections();
	std::string getIkRNS();
	bool setupJointLimits();
	bool writeToPort(yarp::os::Port& port, yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool checkFirstResponseFor1 = true, bool tryReconnect = true );
	bool tryToConnect(std::string &clientName, std::string &serverName);
	bool setupPorts();
	std::string moduleName;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages

	yarp::os::Network yarp;
	yarp::os::Port simoxRobotViewerPort;
	yarp::os::Port simoxIkSolverPort;
	yarp::os::Port simoxMotionPlannerPort;
	yarp::os::Port simoxLegoLocalizerPort;
	yarp::os::Port simoxGraspExecutionPort;

	Eigen::Vector3f fixedHandEyeOffset;
	Eigen::Vector3f fixedHandEyeOffsetRot;
	SimoxGraspingPipelineControlWindowPtr controlWindow;

	std::string currentRNSName;
	std::vector<float> initConfigTorsoArmFingers_rad;

	std::string robotBase; // e.g iCubSim

	bool isLeft;
	std::string eefPreshapeName;

	bool connected;

	float liftHandDeltaMM;

	std::string clientNameGraspExecution;
	std::string serverNameGraspExecution;
	std::string clientNameMotionPlanner;
	std::string serverNameMotionPlanner;
	std::string clientNameRobotViewer;
	std::string serverNameRobotViewer;
	std::string clientNameIkSolver;
	std::string serverNameIkSolver;
	std::string clientNameLegoLocalizer;
	std::string serverNameLegoLocalizer;

	std::string currentObjectName;


	yarp::sig::Vector jointValuesLeftArm,jointValuesRightArm,jointValuesHead,jointValuesTorso;
	yarp::dev::PolyDriver robotDeviceLeftArm,robotDeviceRightArm,robotDeviceHead,robotDeviceTorso;
	yarp::dev::IEncoders *encLeftArm,*encRightArm,*encHead,*encTorso;
	yarp::dev::IPositionControl *posLeftArm,*posRightArm,*posHead,*posTorso;


};


typedef boost::shared_ptr<SimoxGraspingPipelineControlModule> SimoxGraspingPipelineControlModulePtr;

#endif
