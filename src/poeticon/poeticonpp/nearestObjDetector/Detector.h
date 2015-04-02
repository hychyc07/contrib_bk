// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff, Ali Paikan
 * email:  vadim.tikhanoff@iit.it, ali.paikan@iit.it
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

#include <string>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <opencv/cvaux.h>
#include <yarp/sig/Vector.h>

#include <opencv/highgui.h>
#include <opencv/cxcore.h>

class Detector
{ 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> > stereoWorldPort;
    yarp::os::BufferedPort<yarp::os::Bottle>    blobInPort;
    yarp::os::BufferedPort<yarp::os::Bottle>    targetInPort;
    yarp::os::BufferedPort<yarp::os::Bottle>    targetOutPort;

public:

    Detector()
    {
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();

    void loop(); 

    bool interrupt();

protected:

private:

    std::string                 faceExpression;
};

   
   



   
