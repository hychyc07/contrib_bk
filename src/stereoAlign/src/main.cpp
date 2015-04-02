/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
\defgroup icub_stereoAlign stereoAlign
  @ingroup icub_contrib_modules

 This module is responsible for aligning the target image with respect to the reference image. The module should connect to the uncalibrated cameras (left and right), matches features between images, and, using the a reference image, sends out the corrected (in the sense of alignement) image. This can also run using Sift_GPU for better performances.

 \section lib_sec Libraries
 - YARP libraries, OpenCV, Sift_GPU

 \section portsc_sec Ports Created

 - \e /<modName>/imageRef:i receives the uncalibrated reference image.
 
 - \e /<modName>/imageMod:i receives the uncalibrated image that would need to be modified.

 - \e /<modName>/imgRect:o streams out a yarp image correctly aligned with the reference image.

 - \e /<modName>/match:o (optional, see parameters for details) streams out a yarp image containing both original images with the sifts and corresponding matches.
 
 \section parameters_sec Parameters
 
 --name \e name
 - specify the module stem-name, which is
 \e stereoAlign by default. The stem-name is used as
 prefix for all open ports.

 --sendMatch \e value (on/off)
 - specify if the match image should be sent. This sends the image containig sifts and corresponding matches. The default value is off.

 --descriptorRadius \e value
 - specify the descriptorRadius value. This changes the descriptorRadius value for matching sifts. The default value is 200.

 --camDistortionOffset \e value
 - specify the camDistortionOffset value. This changes the camDistortionOffset value for the uncalibrated images. The default value is 3.

 \section parameters_sec Parameters
 - \e /stereoAlign/rpc receives the information to execute some
 manual image modifications. It manages the following commands:
 -# <b>dist</b>: <i>[dist] value</i>. \n
 The value <i>value</i> is represented in pixels and will shift
 the image on the Y axis. \n
 The reply <i>[ok]</i>.
 -# <b>trans</b>: <i>[trans] value</i>. \n
 The value <i>value</i> is represented in pixels and will shift
 the image on the X axis. \n
 The reply <i>[ok]</i>.

 \section tested_os_sec Tested OS
 Windows, Linux, Mac OS

 \section conf_file_sec Configuration Files
 \c config.ini  in \c $ICUB_ROOT/app/stereoAlign/conf \n

 \section example_sec Example Instantiation of the Module

 <tt>stereoAlign --name stereoAlign --context stereoAlign/conf --from config.ini </tt>
 
 Copyright (C) 2009 RobotCub Consortium
 CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \c $ICUB_ROOT/contrib/src/stereoAlign/src/main.cpp

 */

#include "iCub/stereoAlign.h"

/************************************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("stereoAlign/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    StereoAlign stereoAlign;
    return stereoAlign.runModule(rf);
}

//empty line to make gcc happy
