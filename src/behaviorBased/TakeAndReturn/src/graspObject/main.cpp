// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "grasp.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

YARP_DECLARE_DEVICES(icubmod)

class GraspModule: public RFModule
{
   Grasp* grasp;

public:

    GraspModule()
    {
        //period = 1.0;
        grasp = new Grasp();
    }

    ~GraspModule()
    {
        delete grasp;        
    }


    bool configure(ResourceFinder &rf)
    {
        //period = rf.check("period", Value(5.0)).asDouble();
        return grasp->open(rf);
    }

    double getPeriod( )
    {
        return 0.1;        
    }
    
    bool updateModule()
    { 
        grasp->loop();
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        grasp->interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        grasp->close();
        return true;
    }

    //void respond();

private: 
    //double period;
};

int main(int argc, char *argv[]) 
{
    Network yarp;
    YARP_REGISTER_DEVICES(icubmod)

    GraspModule module;
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultConfigFile("grasp_object.ini");
    rf.setDefaultContext("graspObject/conf");
    rf.configure("ICUB_ROOT", argc, argv);

    if (!module.configure(rf))
    {
        fprintf(stderr, "Error configuring module returning\n");
        return -1;
    }
    
    module.runModule();

    printf("Module shutting down\n");

    return 0;
}
