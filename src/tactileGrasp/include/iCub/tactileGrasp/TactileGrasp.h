
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete, Alexander Schmitz
 * email:   andrea.delprete@iit.it, alexander.schmitz@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
*
\defgroup icub_tactileGrasp tactileGrasp
@ingroup icub_contrib_modules

This module can be used for grasping objects exploiting the tactile sensors.

\section intro_sec Description
The module is able to perform four different kinds of grasp using tactile feedback.
Moreover, it also provides to the user a number of operations, useful for tactile grasping.
Here a list of what this module can do: 
-	set the robot arm and hand in a "grasping oriented" position 
-	calibrate the tactile sensors of the hand 
-	grasp an object without using tactile feedback 
-   grasp an object stopping the fingers as soon as a contact is detected 
-   perform a compliant grasp, i.e. the fingers can be moved by touching them, because when touch is detected they move back to the open position
-   run a PID velocity controller that try to keep the touch fixed to the specified value 
-   set the proportional, integrative, derivative gains of the PID controller (also online) 
-   set the desired touch value at which the PID controller aims 


\section lib_sec Libraries
YARP.
SkinDriftCompensation module.

\section parameters_sec Parameters

<b>Command-line Parameters</b> 

The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
(e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below.
 - \c from \c tactileGrasp.ini \n 
   specifies the configuration file
 - \c context \c graspingDemo/conf \n
    specifies the sub-path from \c $ICUB_ROOT/app to the configuration file
 - \c name \c tactileGrasp \n   
    specifies the name of the module (used to form the stem of module port names)  
 - \c robot \c icub \n          
    specifies the name of the robot (used to form the root of robot port names)

<b>Configuration File Parameters </b>
 The following key-value pairs can be specified as parameters in the configuration file 
 (they can also be specified as command-line parameters if you so wish). 
 The value part can be changed to suit your needs; the default values are shown below.
 - \c hand \c right \n    
   specifies which hand has to be used for the grasping
 - \c compensatedTactileDataPort \c /compTactData:i \n  
   specifies the input port name for compensated tactile data (this string will be 
   prefixed by \c /moduleName or whatever else is specified by the name parameter)
 - \c skinDriftCompensationRpcPort \c /calibrationRpcPort:o \n
   specifies the output port name for sending rpc messages to the skinDriftCompensation module
   i.e. commands for calibrating the sensors (this string will be 
   prefixed by \c /moduleName or whatever else is specified by the name parameter)
 
\section portsa_sec Ports Accessed
- /icub/right_arm  or  /icub/left_arm 
- /icub/skin/right_hand_comp  or  /icub/skin/left_hand_comp
- /rightHandSkinDriftComp/rpc  or  /leftHandSkinDriftComp/rpc

\section portsc_sec Ports Created
All the port names listed below will be prefixed by \c /moduleName or whatever else is specified by the name parameter.
\n
<b>Output ports </b>
- /calibrationRpcPort:o : send rpc messages to the rpc port of the skinDriftCompensation module 
	in order to calibrate the tactile sensors and read the percentile values

<b>Input ports: </b>
- /compTactData:i : Vector input port to be connected to the output port of the skinDriftCompensation module, reading compensated tactile data
- /rpc:i: input ports to control the module, accept a yarp::os::Bottle which contains one of these commands:
	- "open hand": set the arm joints to the initial position
	- "calibrate": calibrate the tactile sensors	
	- "grasp tough": grasp without using the tactile feedback
	- "grasp soft": grasp until touch is detected
	- "grasp compliant": grasp when no touch is detected, otherwise open the hand 
	- "control": start the grasp pid controller for controlling the finger positions so as to mantain the desired touch value
	- "stop": stop the grasping (whatever) and reset the arm joint positions
	- "set touch threshold x": set the touch threshold to the specified value x (int in [0,255])
	- "set desired touch x": set the desired touch x that the pid controller must try to mantain (float in [0, 255]) 
	- "set kp i g": set the proportional gain g (float >= 0)for the specified finger i (int in [0,3]) or all the finger if no index is specified
	- "set ki i g": set the integrative gain g (float >= 0)for the specified finger i (int in [0,3]) or all the finger if no index is specified
	- "set kd i g": set the derivative gain g (float >= 0)for the specified finger i (int in [0,3]) or all the finger if no index is specified
	- "help": get a list of the commands accepted by this module
	- "quit": quit the module

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.
 
\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module

tactileGrasp --context graspingDemo/conf --from tactileGraspRight.ini

\author Andrea Del Prete, Alexander Schmitz

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at ICUB_HOME/contrib/src/modules/tactileGrasp/src/TactileGrasp.h.
**/

#ifndef __ICUB_TACTILEGRASP_H__
#define __ICUB_TACTILEGRASP_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include "iCub/tactileGrasp/GraspThread.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

namespace iCub{

namespace tactileGrasp{

class TactileGrasp:public RFModule
{
public:
	// the last element of the enum (COMMANDS_COUNT) represents the total number of commands accepted by this module
	typedef enum { control, open_hand, calibrate, 
		tough_grasp, soft_grasp, compliant_grasp, 
		stop, set_threshold, set_desired_touch, 
		set_KP, set_KI, set_KD, 
		help, quit, COMMANDS_COUNT} TactileGraspCommand;
   
	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(); 
	bool updateModule();
	

private:
	/* module parameters */
	string moduleName;
	string robotName;	
	bool rightHand;						// if true use the right hand, otherwise use the left hand

	/* class constants */
	static const string COMMAND_LIST[];							// list of commands received through the rpc port
	static const string COMMAND_DESC[];							// descriptions of the commands
	
	static const string CALIBRATION_COMMAND[];					// calibration command to send to the skinDriftCompensation	
	static const string GET_PERCENTILE_COMMAND[];				// command to send to the skinDriftCompensation module
																// in order to get the percentile
	static const unsigned int CALIBRATION_COMMAND_LENGTH;		// length of the CALIBRATION_COMMAND array
	static const unsigned int GET_PERCENTILE_COMMAND_LENGTH;	// length of GET_PERCENTILE_COMMAND array

	/* class variables */
	Port skinDriftCompRpcPort;				// output port connected to the skinDriftCompensation module rpc input port	
	Port handlerPort;						// rpc input port to handle commands
	bool percentileHasBeenSet;				// true if the percentile values have been set

	/* pointer to a new thread to be created and started in configure() and stopped in close() */
	GraspThread *graspThread;

	void sendCalibrationMess();
	bool updatePercentile();
	bool identifyCommand(Bottle commandBot, TactileGraspCommand &com);
};

}	// namespace tactileGrasp

}	// namespace iCub

#endif // __ICUB_TACTILEGRASP_H__
//empty line to make gcc happy

