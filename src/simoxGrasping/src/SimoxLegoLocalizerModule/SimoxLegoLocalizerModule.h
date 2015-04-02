#ifndef __Simox_Lego_Localizer_Module_h__
#define __Simox_Lego_Localizer_Module_h__

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/GazeControl.h>

#include <string.h>
#include <vector>

#include <VirtualRobot/VirtualRobot.h>

/*!
    This module can be used to localize Lego models. Currently two models are implemented. 
	For each model custom localization routines have to be implemented.
*/
class SimoxLegoLocalizerModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimoxLegoLocalizerModule();

    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();


    /*
	* The message handler.
    */
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply); 


    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

	enum LegoModel
	{
		eGate,	// the gate
		eXWing,	// the x-wing formed lego
		eNotSet
	};

	void setLegoModel(LegoModel lm);
	std::string getModelString(LegoModel lm);
protected:




	LegoModel legoModel;

	bool setupRobot(const std::string &filename, const std::string &rnsHeadName, const std::string &rnsTorsoName, const std::string &rootName, const std::string &camLeftCoordName, const std::string &camRightCoordName);
	bool setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max );
	bool setJointLimit( const std::string &robotNode, float min, float max );
	/*!
		\param m The resulting pose is stored here [root coord].
		\param update Take earlier localizations into account or start over from scratch.
		\param withObstacle If true an obstacle-blob is assumed to be on the right of the object
		\param storeObstBlob If withObstacle is set, the obstacle blob is stored here.
	*/
	bool localize(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob);
	bool updateRobotModel(bool updateModel);
	bool checkConnections(bool reconnect);
	bool tryToConnect(std::string &clientName, std::string &serverName);
	bool setupPorts();
	bool get3DPoint(float xL, float yL, float xR, float yR, Eigen::Vector3f &store3d);
	bool getGlobalOrientation(float xL, float yL, float aL, float xR, float yR, float aR, const Eigen::Vector3f &p3d_root, Eigen::Matrix4f &storeRot);
	bool createRotatedPose(const Eigen::Vector3f &dir_global, Eigen::Vector3f &dir_locked_global, Eigen::Matrix4f &poseRot);
	bool localizeGate_OneLargeBlob(Eigen::Matrix4f &m);
	bool localizeGate_TwoSmallBlobs(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob);
	bool localizeXWing_TwoSmallBlobs(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob);
	bool filterBlobs(yarp::os::Bottle &b, int nrBlobs);
	Eigen::Vector2f getBlobCenter(yarp::os::Bottle *b);
	
	bool setSegThresh(double t);
	/*!
		Tries to get 3d positions of two blobs.
		stores result in p3d_1_root and p3d_2_root. 
		If update is set, old values are taken into account.
	*/
	bool localizeTwoBlobs(Eigen::Vector3f &p3d_1_root, Eigen::Vector3f &p3d_2_root, bool update, bool withObstacle, Eigen::Matrix4f &storeObstPose);
	bool sendOrientationRobotViewer(Eigen::Vector3f p3d_1_root, Eigen::Vector3f p3d_2_root);
	bool sendObjectRobotViewer(Eigen::Vector3f p_global, Eigen::Vector3f oriDir_global, std::string name);
	std::string moduleName;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages

	double timeUpdateEncodersS;
	double lastEncoderUpdateS;	
	std::string rnsNameTorso,rnsNameHead,rootCoordName,camLeftCoordName,camRightCoordName;
	std::string robotFilename;
	yarp::sig::Vector jointValuesHead,jointValuesTorso;
	yarp::dev::PolyDriver robotDeviceHead,robotDeviceTorso;
	yarp::dev::IEncoders *encHead,*encTorso;
	//! gaze control
	yarp::dev::PolyDriver iCubGazeClient;
	yarp::dev::IGazeControl *iCubGazeControl;

	std::string robotBase; //!< The iCub-robot that is used e.g. icub or icubSim

	VirtualRobot::RobotPtr robot;
	VirtualRobot::RobotNodeSetPtr rnsHead;
	VirtualRobot::RobotNodeSetPtr rnsTorso;
	VirtualRobot::RobotNodePtr rootCoordNode;
	VirtualRobot::RobotNodePtr camLeftCoordNode;
	VirtualRobot::RobotNodePtr camRightCoordNode;

	std::string clientNameDisparity;	
	std::string serverNameDisparity;	
	std::string clientNameBlobExtractorLeft;	
	std::string serverNameBlobExtractorLeft;	
	std::string clientNameBlobExtractorRight;	
	std::string serverNameBlobExtractorRight;	
	std::string clientNameRobotViewer;	
	std::string serverNameRobotViewer;	
	yarp::os::RpcClient stereoDisparityPort;
	yarp::os::RpcClient blobExtractorLeftPort;
	yarp::os::RpcClient blobExtractorRightPort;
	yarp::os::Port simoxRobotViewerPort;

	bool sendResultsToRobotViewer;
	bool canSendResultsToRobotViewer;
	
	// both gate 3d points are stored here
	Eigen::Vector3f localized_p3d_1_root;
	Eigen::Vector3f localized_p3d_2_root;

	yarp::os::Network yarp;
};


typedef boost::shared_ptr<SimoxLegoLocalizerModule> SimoxLegoLocalizerModulePtr;

#endif

