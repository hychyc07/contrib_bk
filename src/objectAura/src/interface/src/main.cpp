// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Katrin Lohan
  * email: katrin.lohan@iit.it
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



#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include "iCub/InterfaceModule.h"


using namespace yarp::os;
using namespace std;
YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[]){

	Network yarp;
    YARP_REGISTER_DEVICES(icubmod)
	InterfaceModule module; 

	ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("");      //overridden by --from parameter
    rf.setDefaultContext("");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

	module.runModule(rf);
	return 0;






}
