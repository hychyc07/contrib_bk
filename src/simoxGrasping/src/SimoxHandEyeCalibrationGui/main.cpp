#include <stdio.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
// Header for the QApplication class
#include <qapplication.h>

#include "SimoxHandEyeCalibrationGui.h"

#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace std;


int main(int argc, char *argv[])
{
    printf("-- START --\n");
    YARP_REGISTER_DEVICES(icubmod)
	yarp::os::Network yarp;
	if (!yarp.checkNetwork())
	{
		cout << "NO CONNECTION TO YARP..." << endl;
		return 1;
	}

	// We must always have an application
	QApplication a( argc, argv );

	/* prepare and configure the resource finder */
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("SimoxHandEyeCalibrationGui_iCub.ini");		//overridden by --from parameter
	rf.setDefaultContext("SimoxHandEyeCalibrationGui/conf");				//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

        bool isleft = false;
        SimoxHandEyeCalibrationGuiPtr rvm(new SimoxHandEyeCalibrationGui(isleft));

	rvm->configure(rf);

	return rvm->runModule();
}

