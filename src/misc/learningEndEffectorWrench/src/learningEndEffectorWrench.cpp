
#include "iCub/learningEndEffectorWrench/learningEndEffectorWrench.h"

/************************************************************************/
learningEndEffectorWrench::learningEndEffectorWrench() { }

/************************************************************************/
bool learningEndEffectorWrench::configure(ResourceFinder &rf) {

    qPort.setSharedArea(&mem);
    fPort.setSharedArea(&mem);
    pThread.setSharedArea(&mem);
    pThread.setOutputPort(&fcPort);

    qPort.open("/learningEndEffectorWrench/q:i");
    fPort.open("/learningEndEffectorWrench/f:i");
    fcPort.open("/learningEndEffectorWrench/fc:o");

    //-----------------CHECK ROBOT AND PART NAME------------//
    ConstString robot_name;
    if (rf.check("robot"))
        robot_name = rf.find("robot").asString();
    else robot_name = DEFAULT_ROBOT;
    ConstString part_name;
    if (rf.check("part"))
        part_name = rf.find("part").asString();
    else part_name = DEFAULT_PART;

    //-----------------CHECK IF AUTOCONNECT IS ON, CONNECT IF TRUE-----------//
    bool autoconnect;
    if (rf.check("autoconnect")) {
        autoconnect = true;
        fprintf(stderr,"Autoconnect option found. \n");
        ConstString bS("/");
        if(Network::connect(bS+robot_name+bS+part_name+"/state:o","/learningEndEffectorWrench/q:i")) {
            mem.setQ(*qPort.read(true));
            printf("[success] connected to %s %s q.\n",robot_name.c_str(),part_name.c_str());
        } else {
            printf("[error] could not connect to %s %s q, aborting...\n",robot_name.c_str(),part_name.c_str());
            return false;
        }
        if(Network::connect(bS+"wholeBodyDynamics/"+part_name+"/endEffectorWrench:o","/learningEndEffectorWrench/f:i")) {
            mem.setF(*fPort.read(true));
            printf("[success] connected to %s external contacts.\n",part_name.c_str());
        } else {
            printf("[error] could not connect to %s external contacts, aborting...\n",part_name.c_str());
            return false;
        }
    } else {
        autoconnect = false;
        fprintf(stderr,"Autoconnect option not found, disabled. \n");
    }

    //-----------------ACTIVATE THE CALLBACKS ALWAYS-----------//
    qPort.useCallback();
    fPort.useCallback();

    pThread.init(rf);

    return true;
}

/*****************************************************************/
double learningEndEffectorWrench::getPeriod() {
    return 5.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool learningEndEffectorWrench::updateModule() {
    printf("Alive\n");
    printf("Latest q: %s\n",mem.getQ().toString().c_str());
    printf("Latest f: %s\n",mem.getF().toString().c_str());
    return true;
}

/************************************************************************/
bool learningEndEffectorWrench::interruptModule() {
    qPort.disableCallback();
    fPort.disableCallback();
    qPort.interrupt();
    fPort.interrupt();
    return true;
}

