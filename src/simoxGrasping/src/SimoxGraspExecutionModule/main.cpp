/* 
 * Copyright (C) 2012 Nikolaus Vahrenkamp
 * Author: Nikolaus Vahrenkamp
 * email: vahrenkamp at users dot sf dot net

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
 \defgroup SimoxGraspExecutionModule SimoxGraspExecutionModule
 @ingroup SimoxGrasping

A module that executes (grasping) motions.

Copyright (C) 2012 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 03/2012

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module uses controles the robot on order to execute a joint space trajectory.


  
\section lib_sec Libraries 
YARP and iCub libraries 

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxGraspExecutionModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub, use 'icub' here.

--executionMode \e modeString
- Specifies which part of iCub is considered. Currently TorsoLeftArm or TorsoRightArm (standard) are supported.

--execution_MaxDelayBetweenPathPoints \e delaySec
- The max delay in seconds that is allowed until a point on the trajectory has to be reached. (standard 5.0)

--execution_switchToNextPathPoint_distDeg \e distDeg
- The orientation max distance (in degree) that has to be reached until the next point on the trajectory can be selected. (standard: 10.0)

--execution_switchToNextPathPoint_distMM \e distMM
- The position max distance (in mm) that has to be reached until the next point on the trajectory can be selected. (standard: 15.0)

--SimoxDataPath \e AdditionalPathToData
- Use this tag to specify one or multiple additional data path(s). Initially the Simox/VirtualRobot/data path is searched for robot models.

--RobotFile \e filename
- The filename for the Simox robot XML definition.

--RobotNodeSet \e name
- The name of the RobotNodeSet (== kinematic chain) that should be considered for execution as defined in the robot's XML file. This definition implicitly selects the number of joints.

--EndEffector \e eef
The end effector definition that should be used (defined in the robot's xml definition)

--EEF_Preshape \e preshape
The name of the EEF preshape that should be used. (When an arm is moved the fingers preshape has influence on the collision checking)

--EnableVisualization \e on/off
- Show the results in an extra RobotViewer window


\section portsc_sec Ports Created

 /stemName/rpc:i The RPC input port.
 
\section in_files_sec Input Data Files

The configuration is read from SimoxGraspExecution/conf/SimoxGraspExecutionModule_iCub.ini (SimoxMotionPlanner/conf/SimoxMotionPlannerModule_iCub_Left.ini)

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include <iostream>

#include "SimoxGraspExecutionModule.h"

using namespace yarp::os;
using namespace std;


YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[])
{
    printf("-- START --\n");

	// we need to initialize the drivers list 
	YARP_REGISTER_DEVICES(icubmod)

	yarp::os::Network yarp;
	if (!yarp.checkNetwork())
	{
		cout << "NO CONNECTION TO YARP..." << endl;
		return 1;
	}

	/* prepare and configure the resource finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("SimoxGraspExecutionModule.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxGraspExecution/conf");						//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxGraspExecutionModulePtr rvm(new SimoxGraspExecutionModule());

	rvm->configure(rf);

	return rvm->runModule();
}

