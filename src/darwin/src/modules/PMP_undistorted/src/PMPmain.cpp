#include "darwin/PMPmodule.h"

using namespace Darwin::pmp;

int main()
{
	Network yarp;
	if (!yarp.checkNetwork())
        return -1;

	PMPmodule myPMPmodule;

	ResourceFinder rf;
	rf.setVerbose(true); // print to a console conf info
	rf.setDefaultConfigFile("PMPundistorted.ini");
	rf.setDefaultContext("PMP_undistorted/conf"); // dove sono i file .ini
	rf.configure("ICUB_ROOT",0,NULL);

	myPMPmodule.runModule(rf);

	return 0;
}