/* 
 * Copyright (C) 2011 Nikolaus Vahrenkamp
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
 \defgroup SimoxIkSolverModule SimoxIkSolverModule
 @ingroup SimoxGrasping

A module for searching IK solutions. The module can handle multiple grasps that have to be defined in a simox XML object definition file (see VirtualRobot::ManipulationObject).

Copyright (C) 2011 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 12/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module performes a reachability check on a set of possible grasps that are defined for a target object. The reachable grasps
can be used to search an IK solution either with the generic IK solver form Simox or with iCub's Cartesian Interface.

  
\section lib_sec Libraries 
YARP libraries and Simox

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxMotionPlannerModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub use 'icub' here.

--side \e left/right
- Which side should be used

--IkMethod \e 0/1
- Which ik solver should be used: 0:Simox ik solver, 1: iCub CartesianControl ikSolver (module must be running!)

--SimoxDataPath \e AdditionalPathToData
- Use this tag to specify onbe or multiple additional data path(s). Initially the Simox/VirtualRobot/data path is searched for robot models.

--RobotFile \e filename
- The filename for the Simox robot XML definition.

--ReachabilityFile \e filename
-The reachability file that should be used. Can be specified relatively to one of Simox' data paths (e.g. .../simox/VirtualRobot/data). The reachability file implictly defines the RobtoNodeSet (== kinematic chain) that is used for Ik solving.

--EndEffector \e eef
The end effector definition that should be used (defined in the robot's xml definition)

--EEF_Preshape \e preshape
The name of the EEF preshape that should be used. (When an arm is moved the fingers preshape has influence on the collision checking)

--EnableVisualization \e on/off
- Show the results in an extra RobotViewer window. Defined in Group "Visualization"

--[CollisionDetection} \e A group defining the collision detection setup
-EnableCD \e on/off Perform collision detection (should be on)
-ConsiderEnvironment \e on/off Consider obstacles that have been added when computing a collision-free motion.
-RobotNodeSet_IkChain \e rnsName The RobtoNodeSet that should be considered for collision detection (must be defined in the robot's xml file).
-RobotNodeSet_Robot \e rnsName The RNS that describes the static part of the robot (self-collision detection)
-SearchColFreeObjectPose \e on/off If on, the object is slightly moved around before start planning in order to find a collision-free target configuration (e.g. needed when the IK solver result is not accurate)


\section portsc_sec Ports Created

 /stemName/rpc:i The RPC input port.
 
\section in_files_sec Input Data Files

The configuration is read from SimoxIkSolver/conf/SimoxIkSolver_iCub_RightHand.ini (SimoxIkSolver/conf/SimoxIkSolver_iCub_LeftHand.ini)

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 



#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include <iostream>

#include "SimoxIkSolverModule.h"

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
	rf.setDefaultConfigFile("SimoxIkSolver_iCub_RightHand.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxIkSolver/conf");							//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxIkSolverModulePtr rvm(new SimoxIkSolverModule());

	rvm->configure(rf);

	return rvm->runModule();
}

