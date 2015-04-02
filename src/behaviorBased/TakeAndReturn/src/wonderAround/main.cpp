// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email:  ali.paikan@iit.it
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


#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "wonder.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define GAZE_TIMER_MAX              5.0
#define GAZE_TIMER_MIN              2.0


class WonderModule: public RFModule
{
   Wonder* wonder;

public:

    WonderModule()
    {
        //period = 1.0;
        wonder = new Wonder();
    }

    ~WonderModule()
    {
        delete wonder;        
    }


    bool configure(ResourceFinder &rf)
    {
        return wonder->open(rf);
    }

    double getPeriod( )
    {
       return Rand::scalar(GAZE_TIMER_MIN,GAZE_TIMER_MAX);
    }
    
    bool updateModule()
    { 
        wonder->loop();
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        wonder->interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        wonder->close();
        return true;
    }

    //void respond();

private: 
    //double period;
};

int main(int argc, char *argv[]) 
{
    Network yarp;
    //YARP_REGISTER_DEVICES(icubmod)

    WonderModule module;
    ResourceFinder rf;
    rf.setVerbose();
    rf.configure("ICUB_ROOT", argc, argv);

   
    if (!module.configure(rf))
    {
        fprintf(stderr, "Error configuring module returning\n");
        return -1;
    }
    
    module.runModule();

    printf("Module shutting down\n");

    return 0;
}
