#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
using namespace yarp::os;

#include "GurlsClassificationBatchModule.h"


int main(int argc, char *argv[]) {
   
    Network yarp;   

    if (!yarp.checkNetwork())
    {
        printf("Could not reach the YARP server\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("GURLSwrapper");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    GurlsClassificationBatchModule mod;

    return mod.runModule(rf);
    
}
