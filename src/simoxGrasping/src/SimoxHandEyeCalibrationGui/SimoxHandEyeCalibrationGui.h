#ifndef __Simox_Hand_Eye_Gui_Module_h__
#define __Simox_Hand_Eye_Gui_Module_h__

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string.h>
#include <vector>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include "SimoxHandEyeCalibrationWindow.h"

/*!
    This GUI can be used to perform hand-eye calibration.
    Therefore a connection is made to SimoxHandtrackerModule and to iCub's Cartesian Interface.
	The GUI itself is handled by a SimoxHandEyeCalibrationWindow object, 
	whereas this class performs the yarp communication.
    The GUI offers the following options:
	1. Set arm configurations 
	2. Set hand preshapes
	3. Show tracking results in HandTrackerModule's viewer
	4. Setup options for HandTrackerModule
	5. Show information about tracking and matching success 

	@see SimoxHandEyeCalibrationWindow
	@see SimoxHandTrackerModule
*/
class SimoxHandEyeCalibrationGui : public yarp::os::RFModule
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
                The constructor.
                \param isLeft Specify if the left or the right hand is used
	*/
        SimoxHandEyeCalibrationGui(bool isLeft);

    double getPeriod();

    /*!
    * Update the GUI class
    */
    bool updateModule();


    /*!
    * Message handler. 
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply); 


    /*!
    * Configure function.
	* Creates GUI class and performs connection setup.
    */
    bool configure(yarp::os::ResourceFinder &rf);

    /*!
    * Interrupt function.
    */
    bool interruptModule();

    /*!
    * Close function, to perform cleanup.
    */
    bool close();

    /*!
		Use Cartesian Interface to move hand to given pose e(x,y,z, axis[3], angle[rad])
    */
    bool moveArm(std::vector<float> handPose_7fd_rootCoords);
    /*!
        Move fingers to given preshape: 9d joint values [degree]
    */
    bool moveFingerJoints(std::vector<float> &fingerJV);

    //! Send a command to hand tracker module
    bool sendToHandTracker(yarp::os::Bottle &cmd,yarp::os::Bottle &response);

    //! The name of the root joint (the base coordinate system)
    std::string getRootName();

    //! The name of the tcp joint (RobotNode as defined in Simox model)
    std::string getTcpName();

    //! The name of the kinematic chain (RobotNodeSet of Simox model)
    std::string getRNSName();

    //! the tcp pose (x,y,z, ax1,ax2,ax3,angle) in m and radian
    std::vector<float> getTCPPosition(bool model);

    //! Retrieve the segmentation threshold form HandTracking module
    float getThreshold();
    
    //! Focus tcp
    void lookToTCP();
	void lookToHomePos();
	void lookToTable();

protected:
    bool setupConnections();
	std::string moduleName;
    std::string handlerPortName;
    yarp::os::Port handlerPort; //a port to handle messages

    yarp::os::Network yarp;
    yarp::os::Port simoxHandTrackerPort;

    SimoxHandEyeCalibrationWindowPtr controlWindow;

    bool isLeft;

    std::string robotBase; // e.g iCubSim

    //! ikSolveriCub
    yarp::dev::PolyDriver         iCubIkSolverClient;
    yarp::dev::ICartesianControl *iCubIkSolverControl;

    //! gaze control
    yarp::dev::PolyDriver iCubGazeClient;
    yarp::dev::IGazeControl *iCubGazeControl;

    std::vector<SimoxHandEyeCalibrationWindow::handPose> handPoses;
    std::vector<SimoxHandEyeCalibrationWindow::preshape> preshapes;
    yarp::sig::Vector jointValuesLeftArm,jointValuesRightArm,jointValuesHead,jointValuesTorso;
    yarp::dev::PolyDriver robotDeviceLeftArm,robotDeviceRightArm,robotDeviceHead,robotDeviceTorso;
    yarp::dev::IEncoders *encLeftArm,*encRightArm,*encHead,*encTorso;
	yarp::dev::IPositionControl *posLeftArm,*posRightArm,*posHead,*posTorso;

};


typedef boost::shared_ptr<SimoxHandEyeCalibrationGui> SimoxHandEyeCalibrationGuiPtr;

#endif
