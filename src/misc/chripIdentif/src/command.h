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

/** 
\defgroup commandGenerator commandGenerator
 
@ingroup icub_tools  
 
@@@TODO
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
@@@TODO
 
\section portsa_sec Ports Accessed
 
@@@TODO
 
\section portsc_sec Ports Created 
 
@@@TODO

\section in_files_sec Input Data Files

@@@TODO

\section out_data_sec Output Data Files 

@@@TODO
 
\section conf_file_sec Configuration Files

@@@TODO

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/DebugInterfaces.h>

#include <math.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

//==============================================================
bool near(double a, double b)
{
    if (abs(a-b)<0.0001) return true;
    else return false;
}

//==============================================================
class CommandClass
{
    public:
    enum
    {
        COMMAND_SQUARE = 0,
        COMMAND_SINE = 1,
        COMMAND_CHIRP = 2,
    };

    public:
    CommandClass(int type, double max, double min, double period);

    double min;
    double max;

    void   start();
    void   update(double* r_command, int* r_cycle, int* r_times, double* r_period);
    void   stop();

    private:
    bool   enable;
    double time_last;
    double time_start;
    double time_current;

    private:
    int    commandType;
    double current_value;
    double period;
    int    ticks;
    int    cycle;
    int    times;
};

CommandClass::CommandClass(int type, double max, double min, double period)
{
    this->commandType = type;
    this->max=max;
    this->min=min;
    this->period=period;
    current_value = 0;
    cycle = 0;
    times = 0;
    ticks = 0;
    time_start = time_current = time_last =Time::now();
}

void CommandClass::start()
{
    time_start = time_current = time_last =Time::now();
    enable = true;
}

void CommandClass::stop()
{
    time_start = time_current = time_last =Time::now();
    enable = false;
}

void CommandClass::update(double* r_command, int* r_cycle, int* r_times, double* r_period)
{
    time_current = Time::now();
    ticks++;
    switch (commandType)
    {
        case COMMAND_SQUARE:
        default:
            if (time_current-time_last>period)
            {
                cycle ++;
                time_last = time_current;
                if      (near(current_value,max))
                        {current_value=min;}
                else if (near(current_value,min))
                        {current_value=max;}
                else    {current_value=max;}
            }
        break;
        case COMMAND_SINE:
            current_value = (max-min)/2 + (max-min)/2 * sin(2*3.14159265/period*(time_current-time_start));

            if (time_current-time_last>period)
            {
                 cycle ++;
                 time_last = time_current;
            }
        break;
        case COMMAND_CHIRP:
            current_value = (max-min)/2 + min + (max-min)/2 * sin(2*3.14159265/period*(time_current-time_start));

            if (time_current-time_last>period)
            {
                 cycle ++;
                 time_last = time_current;
            }

            //if (cycle == 8) //use this if you want the same number of trials in all the frequencies
            if (ticks == 1500) //use this if you want the same duration in all the frequencies
            {
                cycle =0;
                period = period / 1.025;
                time_start = time_current;
                times++;
                ticks=0;
                printf("times: %d period: %f\n", times, period);
            }

            if (period < 0.1) current_value = min;

        break;
    }
    *r_command = current_value;
    *r_cycle = cycle;
    * r_times = times;
    *r_period = period;
}
