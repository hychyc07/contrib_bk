/*
 * Copyright (C) 2013 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * email:  elena.ceseracciu@iit.it
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
#include <yarp/dev/all.h>
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;

/**********************************************************/
int main(int argc, char *argv[])
{
    YARP_REGISTER_DEVICES(icubmod)
    Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout << "YARP server not available!" <<std::endl;
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","dmpExecutor");
    rf.setDefaultConfigFile("dmpExecutor.ini");
    rf.setDefaultContext("DMPgeneralization/conf");
    rf.configure("ICUB_ROOT",argc,argv);

    DmpExecutor manager;
    return manager.runModule(rf);
}



