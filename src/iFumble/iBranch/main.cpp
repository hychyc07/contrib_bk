//jjj : defines an entry point

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <fstream>
#include "fumblyModule.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;

int main(int argc, char *argv[])
{
    Network yarp;	

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("iFumble/conf");
    rf.setDefaultConfigFile("iFumble.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.setDefault("name","iFumble/iFumbly");
    rf.configure("ICUB_ROOT",argc,argv);

    fumblyModule mod;

    return mod.runModule(rf);
}



