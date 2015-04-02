#include "darwin/colorsegModule.h"

using namespace std;
using namespace Darwin::colorseg;

int main(int argc, char* argv[])
{
	Network yarp;
	if (!yarp.checkNetwork())
        return -1;

	colorsegModule colorseg;

	ResourceFinder rf;
	rf.setVerbose(true); // print to a console conf info
	rf.setDefaultConfigFile("colorsegConfiguration.ini");
	rf.setDefaultContext("/conf"); // dove sono i file .ini
	rf.configure("ICUB_ROOT",argc,argv);

	colorseg.runModule(rf);

	return 0;
}
