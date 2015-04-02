/*
 * Copyright (C) 2013 POETICON++, European Commission FP7 project ICT-288382
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */

#include "SequentialLabellerModule.h"

int main(int argc, char *argv[])
{
	/* initialize YARP network */
	Network yarp;
	if(! yarp.checkNetwork() )
		return -1; // EXIT_FAILURE
	
	/* create module */
	SequentialLabellerModule mod;
	
	/* prepare and configure Resource Finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("sequentialLabeller.ini");  // overridden by --from parameter
	rf.setDefaultContext("sequentialLabeller/conf");    // overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
	
	/* runModule calls configure first and, if successful, it then runs the module */
	mod.runModule(rf);
	return 0; // EXIT_SUCCESS
}
