// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: RBCS/IIT && UC3M (c) 2011
 * Author: Juan G. Victores
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "iCub/WBCdemo/WBCdemo.h"

//YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[]) {

    //YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cartesianForceWBC/conf");
    rf.setDefaultConfigFile("cartesianForceWBC.ini");
    rf.configure("ICUB_ROOT", argc, argv);

    Network yarp;
    if (!yarp.checkNetwork()) {
        printf("No yarp network, bye!\n");
        return -1;
    }

    WBCdemo mod;
    return mod.runModule(rf);
}

