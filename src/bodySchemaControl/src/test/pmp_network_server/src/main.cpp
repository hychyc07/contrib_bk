//#include "PMPnetworkModule.h"
#include "pmp_network_server.h"

#include <iostream>
#include <string>

using namespace yarp;
using namespace iCub::pmp_network;

using namespace std;

int main()
{
	Network yarp;
	if (!yarp.checkNetwork())
        return -1;

	
//	PMPmodule myPMPmodule;
	PMPnetwork_server myPMPmodule;

	Property opt;
    opt.put("Context","pmp_network_test/conf");
	myPMPmodule.openInterface(opt);

	return myPMPmodule.runModule();
/*
	ResourceFinder rf;
	rf.setVerbose(true); // print to a console conf info
	rf.setDefaultConfigFile("PMPnetworkConfiguration.ini");
	rf.setDefaultContext("conf/PMP_network"); // dove sono i file .ini
	bool ok = rf.configure("ICUB_ROOT",0,NULL);

	if(ok)	myPMPmodule.runModule(rf);
*/
	//myPMPmodule.test();
}
