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

#ifndef LASER_THREAD_H
#define LASER_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Stamp.h>
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

class LaserThread: public yarp::os::RateThread
{
    protected:
    Property                        iKartCtrl_options;
    ResourceFinder                  &rf;
    PolyDriver                      *laser_driver;
    BufferedPort<yarp::sig::Vector> port_laser_polar_output;
    BufferedPort<yarp::os::Bottle>  port_laser_cartesian_map_output;
    IAnalogSensor                   *iLaser;
    string                          remoteName;
    string                          localName;
    int                             timeout_counter;
    int                             timeout_counter_tot;
    bool                            fake_laser;
    double                          thread_period;
    yarp::os::Stamp                 laserStamp;

    public:
    LaserThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options),
               remoteName(_remoteName), localName(_localName) 
    {
        timeout_counter     = 0;
        timeout_counter_tot = 0;
        if (rf.check("fake_laser")) fake_laser=true;
        else fake_laser = false;
        thread_period = _period;
    }

    virtual bool threadInit();

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Laser thread started successfully\n");
        else
            printf("Laser thread did not start\n");
    }

    virtual void run();

    virtual void threadRelease()
    {
        delete laser_driver;
        port_laser_polar_output.interrupt();
        port_laser_polar_output.close();
        port_laser_cartesian_map_output.interrupt();
        port_laser_cartesian_map_output.close();
    }

    void printStats();
};

#endif