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
 \defgroup SimoxMotionPlannerModule SimoxMotionPlannerModule
 @ingroup SimoxGrasping

A module for planning collision free (grasping) motions.

Copyright (C) 2012 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 01/2012

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module uses Rapidly-exploring Random Trees (RRT) for efficient planning of collision-free motions. 
The setup includes Robot file, kinematic chain, obstacles and start/goal configuration.

Note, that planning is performed in a constrained C-Space in order to forbid regions in which can be dangerous for execution since tendons may break (see iCubConfigurationConstraint).



\section lib_sec Libraries 
YARP libraries and Simox

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxMotionPlannerModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub use 'icub' here.

--PostProcessingSteps \e nrOfPostProcessingLoops
- Specify how many smoothing loops should be performed on the resulting trajectory (standard: 200; 0 to disable)

--SimoxDataPath \e AdditionalPathToData
- Use this tag to specify onbe or multiple additional data path(s). Initially the Simox/VirtualRobot/data path is searched for robot models.

--RobotFile \e filename
- The filename for the Simox robot XML definition.

--RobotNodeSet \e name
- The name of the RobotNodeSet (== kinematic chain) that should be considered for planning as defined in the robot's XML file. This definition implicitly selects the number of joints.

--EndEffector \e eef
The end effector definition that should be used (defined in the robot's xml definition)

--EEF_Preshape \e preshape
The name of the EEF preshape that should be used. (When an arm is moved the fingers preshape has influence on the collision checking)

--EnableVisualization \e on/off
- Show the results in a dedicated RobotViewer window

--[CollisionDetection} \e A group defining the collision detection setup
-EnableCD \e on/off Perform collision detection (should be on)
-ConsiderEnvironment \e on/off Consider obstacles that have been added when computing a collision-free motion.
-RobotNodeSet_robotMove \e rnsName The RobotNodeSet that should be considered for collision detection (must be defined in the robot's xml file).
-RobotNodeSet_robotStatic \e rnsName The RNS that describes the static part of the robot (self-collision detection)
-SearchColFreeObjectPose \e on/off If on, the object is slightly moved around before start planning in order to find a collision-free target configuration (e.g. needed when the IK solver result is not accurate)


\section portsc_sec Ports Created

 <b>Input ports</b>

 - \c /stemName/rpc:i The RPC input port.\n
 
 The RPC port that, used for setup and querying.
   The following commands are available
  
   -  \c help \n
   -  \c quit \n
   -  \c info \n
	List information about internal state
	-  \c add object <name> <filename> \n
	Load object from file <filename> and add as obstacle. Choose <name> as you want, it will be key for further access.
	-  \c remove object <name> \n
	Remove object from internal representation.
	-  \c set object position <name> x y z \n
	Set object <name> to position (x,y,z) (global coordinate system).
	-  \c set object orientation <name> roll pitch yaw \n
	Set RPY orientation of object <name> (global coordinate system).
	-  \c set jointLimits <RNSname> lo_1 hi_1 lo_2 hi_2 ... \n
	Set lower and upper joint limit for RobotNodeSet with name <RNSname> (must be defined in robot's XML file) [rad].
	-  \c get joints \n
	Returns VOCAB_OK and number of joints followed by joint names of current kinematic chain (as defined in robot's XML file).
	-  \c get object position <name> <CoordinateSystem> \n
	Returns VOCAB_OK followed by object's position (x,y,z) in coordinate system of RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coordinate system is used. 
	-  \c get object orientation <name> <CoordinateSystem> \n
	Returns VOCAB_OK followed by object's orientation as three RPY values in coordinate system of robots RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coordinate system is used [rad]. 
	-  \c planMotion  start_1 ... start_i goal_1 ... goal_i \n
	 A collision-free motion is planned for current RobotNodeSet from start to goal configuration. All added objects are considered as obstacles (if not disable din the configuration file). Self-Collisions are considered as defined in the configuration file.
	 On success (first result=VOCAB_OK), the path in configuration space is returned as set of configs ((start1 ... start_i) (c1_1 ... c1_i) ... (cn_1 ... cn_i) (goal_1 ... goal_i)).


 
\section in_files_sec Input Data Files

The configuration is read from SimoxMotionPlanner/conf/SimoxMotionPlannerModule_iCub_Right.ini (SimoxMotionPlanner/conf/SimoxMotionPlannerModule_iCub_Left.ini)

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include <iostream>

#include "SimoxMotionPlannerModule.h"

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
	rf.setDefaultConfigFile("SimoxMotionPlannerModule_iCub_Right.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxMotionPlanner/conf");						//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxMotionPlannerModulePtr rvm(new SimoxMotionPlannerModule());

	rvm->configure(rf);

	return rvm->runModule();
}

