/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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


#include <yarp/os/Network.h>
#include "iCub/module.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;


/**********************************************************/
int main(int argc, char *argv[])
{

    YARP_REGISTER_DEVICES(icubmod)
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","karmaManager");
    rf.setDefaultContext("karma/conf");
    rf.setDefaultConfigFile("karmaManager.ini");
    rf.setDefault("tracking_period","30");
    rf.configure("ICUB_ROOT",argc,argv);

    Manager manager;
    return manager.runModule(rf);
}



