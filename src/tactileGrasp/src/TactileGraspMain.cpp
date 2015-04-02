
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

#include "iCub/tactileGrasp/TactileGrasp.h" 

int main(int argc, char * argv[])
{
	/* initialize yarp network */ 
	Network yarp;
	//Network::init();
	//Time::turboBoost();

	/* create your module */
	iCub::tactileGrasp::TactileGrasp module;

	/* prepare and configure the resource finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("tactileGrasp.ini");			//overridden by --from parameter
	rf.setDefaultContext("tactilegraspingDemo/conf");		//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	/* run the module: runModule() calls configure first and, if successful, it then runs */
	module.runModule(rf);

	return 0;
}
