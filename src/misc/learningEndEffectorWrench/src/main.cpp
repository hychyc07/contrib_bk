/*
 * Copyright: RBCS/IIT && UC3M (c) 2011
 * Author: Juan G. Victores
 * Contrib: Code and docs from A. Gijsbert, namely ${ICUB_ROOT}/main/src/modules/learningMachine
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <yarp/os/all.h>

#include "iCub/learningEndEffectorWrench/learningEndEffectorWrench.h"


using namespace yarp::os;
using namespace std;



int main(int argc, char** argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("learningEndEffectorWrench/conf");
    rf.setDefaultConfigFile("learningEndEffectorWrench.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help")) {
        fprintf(stdout,"Options:\n");
        fprintf(stdout,"\t--robot   robot: robot (default: \"icub\")\n");
        fprintf(stdout,"\t--part    part: part (default: \"left_arm\")\n");
        fprintf(stdout,"\t--rate    rate: ms (default: \"20\")\n");
        return 0;
    }

    Network yarp;
    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }

    learningEndEffectorWrench mod;
    return mod.runModule(rf);

    return 0;
}

