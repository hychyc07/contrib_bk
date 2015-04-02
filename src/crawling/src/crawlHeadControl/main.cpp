/* 
 * Copyright (C) 2008 Sebastien GAY
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sebastien.gay@epfl.ch
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
 * @ingroup icub_crawling
 *
 * \defgroup icub_crawlHeadControl crawlHeadControl
 *
 *This module is part of the application \ref icub_crawling "crawling"
 *
 *\section intro_sec Description 
 *
 * This module controls the head of the robot when approaching a "good" object.
 * It gets the position of one or more objects and decides which one to focus on (the nearest good one).
 * I makes sure that the object stays in the field of view of the robot.
 * It outputs the joint configuration of the head to achieve this.
 *
 *\section lib_sec Libraries
 *
 * This module requires no additional library.
 *
 *\section parameters_sec Parameters
 *
 * The config file (default is  $ICUB_ROOT/app/crawling/config/headControl/config.ini). call with :
 *   --file  [path to config file].ini
 *
 *
 *
 *\section portssa_sec Ports Accessed 
 *
 * 
 * <ul>
 * <li> all ports of the iCub Interface for the head should be running for the module to  
 * get the current joints angles of the head.
 * </ul>
 *
 *\section portsc_sec Ports Created
 *
 * </ul>
 * Input ports\n
 * <ul>
 * <li> /crawlHeadControl/in to receive the 3D position of the objects.
 *</ul>
 * Output ports\n
 * <ul>
 * <li> /crawlHeadControl/out to output the head angles.
 * </ul>
 *
 *\section conf_file_sec Configuration Files
 *
 *This module requires a config file :
 *<ul>
 *<li> config.ini
 *</ul>
 *
 * It can be found at \in $ICUB_ROOT/app/crawling/config/tracker/config.ini 
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * crawlHeadControl --file "C:\iCub\app\crawling\config\headControl\config.ini"

 * This file can be edited at \in src/crawling/crawlHeadControl/main.cpp 
 *
 *\authors Sebastien GAY
 *
 **/

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


#include "crawlHeadControl.h"

int main(int argc, char *argv[])
{
    /* Initialize and check yarp network */
	Network yarp;
	
    if (!yarp.checkNetwork())
        return -1;
	
	/* Create the crawlHeadControl module*/
	crawlHeadControl headControlModule;
	
	/* Prepare and configure the resource finder (added CT)*/
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("config.ini"); // ICUB_ROOT\contrib\src\crawlingTest\src\crawlHeadControl\release\config.ini
	rf.setDefaultContext("crawlingdemo/conf/headControl");
	rf.configure("ICUB_ROOT", argc, argv);

	/* Run the module with rf */
	//return headControlModule.runModule(argc, argv);;
	headControlModule.runModule(rf);
	return 0;

}


      
