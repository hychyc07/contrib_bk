#include <iostream>

#include <yarp/os/Network.h> 
#include "darwin/DevDriverModule.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace Darwin::pmp;

int main()
{
	Network::init();

	DevDriverModule dev;
	ResourceFinder rf;

	rf.setVerbose(true); // print to a console conf info
        rf.setDefaultConfigFile("DevDriverConfiguration.ini");
        rf.setDefaultContext("LowLevelController_DARWIN/conf"); // dove sono i file .ini
	rf.configure("ICUB_ROOT",0,NULL);

	if(!dev.configure(rf))
	{
		fprintf(stderr,"\nError configuring module");
		return -1;
	}

	dev.runModule();

	Network::fini();

	return 0;
}
