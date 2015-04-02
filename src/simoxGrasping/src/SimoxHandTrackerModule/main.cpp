#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include <iostream>

#include "SimoxHandTrackerModule.h"

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
    rf.setDefaultConfigFile("SimoxHandTrackerModule_iCub_right.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxHandTrackerModule/conf");			//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	SimoxHandTrackerModulePtr rvm(new SimoxHandTrackerModule());

	rvm->configure(rf);
        cout << "STARTING MODULE..."<<endl;

	return rvm->runModule();
}

