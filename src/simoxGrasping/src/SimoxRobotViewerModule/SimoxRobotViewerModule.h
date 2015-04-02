#ifndef __Simox_Robot_Viewer_Module_h__
#define __Simox_Robot_Viewer_Module_h__

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string.h>
#include <vector>

#include <VirtualRobot/VirtualRobot.h>
#include "../SimoxRobotViewer/SimoxRobotViewer.h"

/*!
    This module can be used to visualize the current state of the iCub. 
	Therefore iCub's joint values are queried and sent to the Simox robot viewer window.
*/
class SimoxRobotViewerModule : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimoxRobotViewerModule();

    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();

	//int runModule();

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

protected:

	enum visuModeState
	{
		eShowCurrentRobotState,
		eShowStaticConfig
	};

	visuModeState visuMode;

	// mapping ot simox finger joints
	std::vector<float> getFingerJoints(yarp::sig::Vector &enc);

	std::string moduleName;
	std::string handlerPortName;
	yarp::os::Port handlerPort; //a port to handle messages

	double timeUpdateEncodersS;
	double lastEncoderUpdateS;	
	std::string rnsArmLeft,rnsArmRight,rnsHandLeft,rnsHandRight,rnsTorso,rnsLegLeft,rnsLegRight,rnsHead;
	yarp::sig::Vector jointValuesLeftArm,jointValuesRightArm,jointValuesHead,jointValuesTorso;
	yarp::dev::PolyDriver robotDeviceLeftArm,robotDeviceRightArm,robotDeviceHead,robotDeviceTorso;
	yarp::dev::IEncoders *encLeftArm,*encRightArm,*encHead,*encTorso;

	bool updateFingersLeft,updateFingersRight;

	SimoxRobotViewerPtr viewer;
	std::string robotBase; //!< The iCub-robot that is used e.g. icub or icubSim
};


typedef boost::shared_ptr<SimoxRobotViewerModule> SimoxRobotViewerModulePtr;

#endif

