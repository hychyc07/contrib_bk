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
 \defgroup SimoxRobotViewerModule SimoxRobotViewerModule
 @ingroup SimoxGrasping

A module that shows the current state of the robot in a 3D window.

Copyright (C) 2011 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 11/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module loads a Simox robot model of iCub and continuously transfers current joint values to the model.
The model is shown in a 3D window, based on Coin3D/Qt.

  
\section lib_sec Libraries 
YARP libraries and Simox

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxRobotViewerModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub use 'icub' here.

\section portsc_sec Ports Created
 
/stemName/rpc:i The RPC input port.

Connections are made to iCub's Cartesian Inetrface to left and right arm, head and torso

\section in_files_sec Input Data Files

The configuration is read from SimoxRobotViewer/conf/SimoxRobotViewer_iCub.ini

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include "SimoxRobotViewerModule.h"

using namespace yarp::os;
using namespace std;

//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>



int main(int argc, char *argv[])
{
    printf("-- START --\n");
	yarp::os::Network yarp;
	if (!yarp.checkNetwork())
	{
		cout << "NO CONNECTION TO YARP..." << endl;
		return 1;
	}

	/* prepare and configure the resource finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("SimoxRobotViewer_iCub.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxRobotViewer/conf");				//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxRobotViewerModulePtr rvm(new SimoxRobotViewerModule());

	rvm->configure(rf);

	bool res = rvm->runModule();

	rvm.reset();
	//_CrtDumpMemoryLeaks();

	return res;
}

