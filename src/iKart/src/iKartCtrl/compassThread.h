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

#ifndef COMPASS_THREAD_H
#define COMPASS_THREAD_H

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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class CompassThread: public yarp::os::RateThread
{
    protected:
    Property iKartCtrl_options;

    ResourceFinder      &rf;
    BufferedPort<yarp::sig::Vector> port_inertial_input;
    BufferedPort<yarp::sig::Vector> port_compass_output;
    yarp::sig::Vector compass_data;
    yarp::sig::Vector inertial_data;
    string remoteName;
    string localName;
    int                 timeout_counter;

    public:
    
    CompassThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options),
               remoteName(_remoteName), localName(_localName) 
    {
        timeout_counter     = 0;
        inertial_data.resize(12,0.0);
        compass_data.resize(3,0.0);
    }

    virtual bool threadInit()
    {
        port_inertial_input.open((localName+"/inertial:i").c_str());
        port_compass_output.open((localName+"/compass:o").c_str());
        Network::connect("/icub/inertial",(localName+"/inertial:i").c_str());
        return true;
    }

    virtual void run();

    virtual void threadRelease()
    {    
        port_inertial_input.interrupt();
        port_inertial_input.close();
        port_compass_output.interrupt();
        port_compass_output.close();
    }

    void printStats();

};

#endif