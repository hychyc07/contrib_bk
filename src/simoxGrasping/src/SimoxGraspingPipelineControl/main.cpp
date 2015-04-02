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
\defgroup SimoxGraspingPipelineControlModule Simox Grasping Pipeline Control Module
@ingroup SimoxGrasping

A control GUI for performing grasping tasks.

Copyright (C) 2011 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 11/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module offers a GUI to control grasping tasks, based on Simox.

  
\section lib_sec Libraries 
YARP libraries and Simox

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxGraspingPipelineControlModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub use 'icub' here.

\section portsc_sec Ports Created

 /stemName/rpc:i The RPC input port.
 
\section in_files_sec Input Data Files
None. 

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
// Header for the QApplication class
#include <qapplication.h>


#include "SimoxGraspingPipelineControlModule.h"

#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[])
{
    printf("-- START --\n");
	yarp::os::Network yarp;
	if (!yarp.checkNetwork())
	{
		cout << "NO CONNECTION TO YARP..." << endl;
		return 1;
	}

	YARP_REGISTER_DEVICES(icubmod)

	// We must always have an application
	QApplication a( argc, argv );

	/* prepare and configure the resource finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("SimoxGraspingPipelineControlGui_iCub_RightHand.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxGraspingPipeline/conf");									//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxGraspingPipelineControlModulePtr rvm(new SimoxGraspingPipelineControlModule());

	rvm->configure(rf);

	return rvm->runModule();
}

