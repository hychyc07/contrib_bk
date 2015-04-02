/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
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

/**
 \defgroup karmaToolProjection Tool Projection Part of the KARMA Experiment

 @ingroup icub_karma

 This module finds the tooltip using motionCUT.

 \section intro_sec Description
 Through an active exploration of the tool held in the robot hand
 this module extract a set of points for the tooltip location in the image.

 \section lib_sec Libraries
 - YARP libraries.
 - OpenCV library.

 \section parameters_sec Parameters

 none

 \section portsc_sec Ports Created
 - \e /karmaToolFinder/rpc 
 not responding to anything.
 
 - \e /karmaToolProjection/motionFilter:i receives the data blobs from the motionCUT module
 
 - \e /karmaToolProjection/target:o streams the X and Y target points of the tooltip 
 
 - \e /karmaToolProjection/img:o streams the image with the image analysis
 

 \section tested_os_sec Tested OS
 Windows, Linux, MacOS

 \author Vadim Tikhanoff
 
 */
#include <yarp/os/Network.h>
#include "iCub/module.h"

using namespace yarp::os;

/**********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","karmaToolProjection");
    rf.setDefault("tracking_period","30");
    rf.configure("ICUB_ROOT",argc,argv);

    Manager manager;
    return manager.runModule(rf);
}

