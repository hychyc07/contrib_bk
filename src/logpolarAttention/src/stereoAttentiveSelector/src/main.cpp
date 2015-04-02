// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @brief main code for the stero attentive module; this is part of the stereo attententive system.
 */

#include "iCub/stereoAttModule.h" 
//#include <ippi.h>
//#include <ippcore.h>
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    /* important, this effectively disables the OMP library parallelization in the IPP */
    //ippSetNumThreads(1);
    Network yarp;
    Time::turboBoost();
    stereoAttModule module; 
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("stereoAttModule.ini"); //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    module.runModule(rf);
    return 0;
}


