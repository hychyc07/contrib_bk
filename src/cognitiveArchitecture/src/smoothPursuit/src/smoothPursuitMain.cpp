/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.robotcub.org & www.vernon.eu
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
 
/*
Audit Trail
-----------

22/12/11  Started work on the development of this module for Rong Li   DV
*/ 
 
#include "iCub/smoothPursuit.h"

 
int main(int argc, char *argv[]) {

    /* initialize yarp network */

    Network yarp;

    /* create the module */

    SmoothPursuit module;

    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("smoothPursuit.ini");  
    rf.setDefaultContext("smoothPursuit/conf");  // the default path will now be $ICUB_ROOT/app/smoothPursuit/conf              
    rf.configure("ICUB_ROOT", argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */

    return module.runModule(rf);
}

/* empty line to make gcc happy */


