/* 
* Copyright (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
* Author: Ilaria Gori, Elena Ceseracciu
* email:   
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
#include <yarp/os/ResourceFinder.h>

#include "objectReconstr.h"

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("objectReconstr/conf");
    rf.configure("ICUB_ROOT",argc,argv);

    ObjectReconstr mod;

    return mod.runModule(rf);
}


