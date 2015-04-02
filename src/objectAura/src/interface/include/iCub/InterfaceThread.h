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
#include <yarp/sig/all.h>

#include <stdio.h>
#include <iostream>
#include <yarp/dev/all.h>
#include "iCub/detectcollision.h"
#include "iCub/anticipation.h"
#include "iCub/objecthandler.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
  
class InterfaceThread : public Thread
{
private:

   /* class variables */

    bool oA;
    bool oB;
    bool oC;
    int *thresholdValue;  
    BufferedPort<Bottle> *port1, *port2, *port3, *port4, *outport, *outport2; 
     void bodyBottle(Bottle *outport, Bottle *outport2);
    void bucketBottle(Bottle *outport, Bottle *outport2);
    void handBottle(Bottle *outport, Bottle *outport2);      
    void trackBallOneBottle(Bottle *outport, Bottle *outport2);
    void trackBallTwoBottle(Bottle *outport, Bottle *outport2);
    void trackBallThreeBottle(Bottle *outport, Bottle *outport2);
    void unfixedBottle(Bottle *outport, Bottle *outport2);
    void startBottle(Bottle *outport, Bottle *outport2);
        bool endoftrail;
   
public:

   /* class methods */

   InterfaceThread(BufferedPort<Bottle> *port1, BufferedPort<Bottle> *port2, BufferedPort<Bottle> *port3, BufferedPort<Bottle> *port4, BufferedPort<Bottle> *outport, BufferedPort<Bottle> *outport2);
   bool threadInit();     
   void threadRelease();
   void run(); 
    
};

