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
 \defgroup SimoxLegoLocalizerModule SimoxLegoLocalizerModule
 @ingroup SimoxGrasping

A module for localizing lego models.

Copyright (C) 2012 Nikolaus Vahrenkamp
 
Author: Nikolaus Vahrenkamp
 
Date: first release on 02/2012

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module searches for lego models in the current camera images. The orientation is computed by merging the 2d orientations
of the corresponding blobs (the models need to have a long segmentable brick for this) and the 3D position is retrieved by 
stereo matching.

  
\section lib_sec Libraries 
YARP libraries and Simox

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. Standard: 'SimoxLegoLocalizerModule'.

--robot \e robot
- The name of the robot. Standard: 'icubSim'. When using this module on iCub use 'icub' here.

\section portsc_sec Ports Created

 
\section in_files_sec Input Data Files

The configuration is read from SimoxLegoLocalizer/conf/SimoxLegoLocalizer_iCub.ini

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux.
*/ 

#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include "SimoxLegoLocalizerModule.h"

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
	rf.setDefaultConfigFile("SimoxLegoLocalizer_iCub.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxLegoLocalizer/conf");				//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxLegoLocalizerModulePtr rvm(new SimoxLegoLocalizerModule());

	rvm->configure(rf);

	return rvm->runModule();
}

