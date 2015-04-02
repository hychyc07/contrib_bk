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

#include "laserThread.h"
#define _USE_MATH_DEFINES
#include <math.h>

const double laser_tf = 0.245; //m
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

bool LaserThread::threadInit()
{
    ConstString laser_filename = iKartCtrl_options.findGroup("GENERAL").find("laser").asString();
    ConstString laser_config_filename =rf.findFile(laser_filename);        
    if (laser_config_filename=="") 
    {
        printf("\nError! Unable to locate .ini laser configuration file. \nLooking for %s\n",laser_config_filename.c_str());
        return false;
    }
    else
    {
        printf("\nOpening the laser interface...\n");
        Property laser_options;
        laser_options.fromConfigFile(laser_config_filename.c_str());
        laser_options.put("CONFIG_PATH",rf.getContextPath().c_str());

        // open the laser scanner driver
        laser_driver=new PolyDriver;
        laser_options.put("device","laserHokuyo");
        if (fake_laser) laser_options.put("fake","");
        if (!laser_driver->open(laser_options))
        {
            fprintf(stderr,"ERROR: cannot open laser driver...\n");
            delete laser_driver;    
            return false;
        }
        //open the interface for the laser
        bool laser_ok = laser_driver->view(iLaser);
        if(!laser_ok)
        {
            fprintf(stderr,"ERROR: cannot view the laser interface\nreturning...\n");
            return false;
        }
        // open the laser output port
        port_laser_polar_output.open((localName+"/laser:o").c_str());
        port_laser_cartesian_map_output.open((localName+"/laser_map:o").c_str());
    }

    return true;
}

void LaserThread::run()
{
    yarp::sig::Vector laser_data;
    //fprintf(stderr,"before laser reading\n");
    double before_laser=Time::now();
    int res = iLaser->read(laser_data);
    laserStamp.update();
    double after_laser=Time::now();
    if (after_laser-before_laser > 0.040) { timeout_counter++; timeout_counter_tot++;}
    //fprintf(stderr,"after laser reading\n");
    if (res == yarp::dev::IAnalogSensor::AS_OK)
    {
        if (port_laser_polar_output.getOutputCount()>0)
        {
            yarp::sig::Vector &plaser_data=port_laser_polar_output.prepare();
            plaser_data=laser_data;
            port_laser_polar_output.setEnvelope(laserStamp);
            port_laser_polar_output.write();
        }
        if (port_laser_cartesian_map_output.getOutputCount()>0)
        {
            yarp::os::Bottle &plaser_data=port_laser_cartesian_map_output.prepare();
			plaser_data.clear();
            for (unsigned int i=0; i<laser_data.size(); i++)
            {
                yarp::os::Bottle b;
                double alpha = ((1080-i)/1080.0*270.0-135.0)*DEG2RAD;
                double y = laser_data[i]*cos(alpha)+laser_tf;
                double x = laser_data[i]*sin(alpha);
                b.addDouble(x);
                b.addDouble(y);
                plaser_data.addList() = b;
            }
            port_laser_cartesian_map_output.setEnvelope(laserStamp);
            port_laser_cartesian_map_output.write();
        }
    }
    else
    {
        fprintf(stderr,"Error reading laser data, code: %d\n", res);
    }
}

void LaserThread::printStats()
{
    double max_tpt = 1000/thread_period;
    fprintf (stdout,"* Laser thread:\n");
    fprintf (stdout,"timeouts: %3d(%3d) tot: %3d\n",timeout_counter,int(max_tpt), timeout_counter_tot);
    timeout_counter=0;
}
