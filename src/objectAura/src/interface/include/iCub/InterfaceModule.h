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



#include <iostream>
#include <string>
#include <stdio.h>
#include <cstring>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "iCub/InterfaceThread.h"

using namespace std;
using namespace yarp::os; 

class InterfaceModule:public RFModule
{
   /* module parameters */
	string moduleName;
	string robotName;
	string robotPortName;
	string inputPortName1;
    string inputPortName2;
    string inputPortName3;
    string inputPortName4;
	string outputPortName;
    string outputPortName2;
 
   /* class variables */
	InterfaceThread* interface;

    /*Ports*/
    BufferedPort<Bottle> port1;
    BufferedPort<Bottle> port2;
    BufferedPort<Bottle> port3;
    BufferedPort<Bottle> port4;

    BufferedPort<Bottle> outport;
    BufferedPort<Bottle> outport2;

public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond();
   double getPeriod(); 
   bool updateModule();
};


