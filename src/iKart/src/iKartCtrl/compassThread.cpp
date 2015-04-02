/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
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
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>

#include "compassThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

void CompassThread::run()
{
    yarp::sig::Vector *iner = port_inertial_input.read(false);
    if (iner) inertial_data = *iner;
    else timeout_counter++;

    //add here kinematics computation
    compass_data[0]=inertial_data[2];
    compass_data[1]=inertial_data[1];
    compass_data[2]=inertial_data[0];

    yarp::sig::Vector &pcompass_data=port_compass_output.prepare();
    pcompass_data=compass_data;
    //lastStateStamp.update();
    //port_compass_data.setEnvelope(lastStateStamp);
    port_compass_output.write();
}


void CompassThread::printStats()
{
    fprintf (stdout,"* Compass thread:\n");
    fprintf (stdout,"timeouts: %d\n",timeout_counter);
}
