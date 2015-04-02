#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "powerGrasp.h"

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("powerGrasp/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    PowerGrasp mod;

    return mod.runModule(rf);
}

