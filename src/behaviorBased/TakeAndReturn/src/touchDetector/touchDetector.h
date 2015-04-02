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

#include <string>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>


YARP_DECLARE_DEVICES(icubmod)

class TouchDetector
{
   
public:

    TouchDetector()
    {
       // constructor
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();

    void loop(); 

    bool interrupt();


private: 
    bool detectContact(iCub::skinDynLib::skinContactList *_sCL);
    yarp::sig::Vector locateContact(iCub::skinDynLib::skinContact &sc);

private:
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> inputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> faceExpPort;
    std::string faceExpression;

    yarp::sig::Vector cntctPosWRF;      // Position in WRF
    std::string cntctArm;         // Affected Arm
    iCub::skinDynLib::skinContact cntctSkin;   // SkinContact

    // Driver for "classical" interfaces
    yarp::dev::PolyDriver       ddR; // right arm device driver
    yarp::dev::PolyDriver       ddL; // left arm  device driver
    yarp::dev::PolyDriver       ddT; // torso controller  driver

    yarp::dev::IEncoders    *iencsR;
    yarp::sig::Vector              *encsR;
    yarp::dev::IEncoders         *iencsL;
    yarp::sig::Vector            *encsL;
    yarp::dev::IEncoders         *iencsT;
    yarp::sig::Vector            *encsT;   
};

   
   



   
