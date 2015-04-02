// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
  
/**
 * @file main.cpp
 * @brief main code for the mosaic module; this is part of the logpolar attention system.
 */

#include "iCub/mosaicModule.h" 
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[]){

    Network yarp;
    mosaicModule module; 
    YARP_REGISTER_DEVICES(icubmod)
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("mosaic.ini");      //overridden by --from parameter
    rf.setDefaultContext("logpolarAttention/conf");    //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);  
 
    module.runModule(rf);
    return 0;
}


