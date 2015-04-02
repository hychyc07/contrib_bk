#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include "iCub/skinCalibration/CalibrationModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinCalibration;

int main(int argc, char **argv){

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	CalibrationModule cm;

	ResourceFinder rf;
	rf.setVerbose(false);
	rf.setDefaultConfigFile("skinCalibration.ini");		//overridden by --from parameter
	rf.setDefaultContext("skinCalibration/conf");				//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);

	cm.runModule(rf);  

    return 0;

}