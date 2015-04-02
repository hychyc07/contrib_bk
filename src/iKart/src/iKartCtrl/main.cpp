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
\defgroup iKartCtrl iKartCtrl
 
@ingroup icub_module  
 
IKart controller (wheels decoupling prototype).
 
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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "controlThread.h"
#include "laserThread.h"
#include "compassThread.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr;
    LaserThread    *laser_thr;
    CompassThread  *compass_thr;
    Port            rpcPort;

public:
    CtrlModule() 
    {
        control_thr=0;
        laser_thr=0;
        compass_thr=0;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("ctrlName",Value("ikart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;
        
        //reads the configuration file
        Property iKartCtrl_options;

        ConstString configFile=rf.findFile("from");
        if (configFile=="") //--from iKartCtrl.ini
        {
            printf("\nError! Cannot find .ini configuration file. \nBy default I'm searching for iKartCtrl.ini\n");
            return false;
        }
        else
        {
            iKartCtrl_options.fromConfigFile(configFile.c_str());
        }

        iKartCtrl_options.put("remote",remoteName.c_str());
        iKartCtrl_options.put("local",localName.c_str());

        //set the thread rate
        int rate = rf.check("rate",Value(20)).asInt();
        printf("\niKartCtrl thread rate: %d ms.\n",rate);

        // the motor control thread
        bool motors_enabled=true;
        if (rf.check("no_motors"))
        {
            printf("\n'no_motors' option found. Skipping motor control part.\n");
            motors_enabled=false;
        }

        if (motors_enabled==true)
        {
            control_thr=new ControlThread(rate,rf,iKartCtrl_options);
            if (!control_thr->start())
            {
                delete control_thr;
                return false;
            }
        }

        // the laser thread
        bool laser_enabled=true;
        if (iKartCtrl_options.findGroup("GENERAL").check("laser")==false)
        {
            printf("\nLaser configuration not specified. Turning off laser.\n");
            laser_enabled=false;
        }
        if (rf.check("no_laser"))
        {
            printf("\nLaser disabled.\n");
            laser_enabled=false;
        }

        if (laser_enabled==true)
        {
            laser_thr=new LaserThread(rate,rf,iKartCtrl_options,remoteName,localName);
            if (!laser_thr->start())
            {
                delete laser_thr;
                return false;
            }
        }

        // the compass thread
        bool compass_enabled=true;
        if (rf.check("no_compass"))
        {
            printf("\n'no_compass' option found. Skipping inertial/compass part.\n");
            compass_enabled=false;
        }

        if (compass_enabled==true)
        {
            compass_thr=new CompassThread(rate,rf,iKartCtrl_options,remoteName,localName);
            if (!compass_thr->start())
            {
                delete compass_thr;
                return false;
            }
        }

        //try to connect to joystickCtrl output
        if (rf.check("joystick_connect"))
        {
            int joystick_trials = 0; 
            do
            {
                yarp::os::Time::delay(1.0);
                if (yarp::os::Network::connect("/joystickCtrl:o","/ikart/joystick:i"))
                    {
                        printf("Joystick has been automaticallly connected\n");
                        break;
                    }
                else
                    {
                        printf("Unable to find the joystick port, retrying (%d/5)...\n",joystick_trials);
                        joystick_trials++;
                    }

                if (joystick_trials>=5)
                    {
                        printf("Unable to find the joystick port, giving up\n");
                        break;
                    }
            }
            while (1);
        }

        //check for debug mode
        if (rf.check("debug"))
        {
            this->control_thr->enable_debug(true);
        }

        rpcPort.open((localName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("run");
            reply.addString("idle");
            reply.addString("reset_odometry");
            reply.addString("set_prefilter 0/1/2/4/8");
            reply.addString("set_motors_filter 0/1/2/4/8");
            reply.addString("change_pid <identif> <kp> <ki> <kd>");
            reply.addString("change_ctrl_mode <type_string>");
            reply.addString("set_debug_mode 0/1");
            return true;
        }
        else if (command.get(0).asString()=="set_debug_mode")
        {
            if (control_thr)
            {
                if (command.get(1).asInt()>0)
                    {control_thr->enable_debug(true); reply.addString("debug mode on");}
                else
                    {control_thr->enable_debug(false); reply.addString("debug mode off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="set_prefilter")
        {
            if (control_thr)
            {
                if (command.get(1).asInt()>0) 
                    {control_thr->set_input_filter(command.get(1).asInt()); reply.addString("Prefilter on");}
                else
                    {control_thr->set_input_filter(0); reply.addString("Prefilter off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="set_motors_filter")
        {
            if (control_thr)
            {
                if (command.get(1).asInt()>0) 
                    {control_thr->get_motor_handler()->set_motors_filter(command.get(1).asInt()); reply.addString("Motors filter on");}
                else
                    {control_thr->get_motor_handler()->set_motors_filter(0); reply.addString("Motors filter off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="run")
        {
            if (control_thr)
            {
                if (control_thr->get_motor_handler()->turn_on_control())
                    {reply.addString("Motors now on");}
                else
                    {reply.addString("Unable to turn motors on! fault pressed?");}

            }
            return true;
        }
        else if (command.get(0).asString()=="idle")
        {
            if (control_thr)
            {
                control_thr->get_motor_handler()->turn_off_control();
                {reply.addString("Motors now off.");}
            }
            return true;
        }
        else if (command.get(0).asString()=="change_ctrl_mode")
        {
            if (control_thr)
            {
                if (control_thr->set_control_type(command.get(1).asString().c_str()))
                    {reply.addString("control mode changed");}
                else
                    {reply.addString("invalid control mode request");}
            }
            return true;
        }
        else if (command.get(0).asString()=="change_pid")
        {
            if (control_thr)
            {
                string identif = command.get(1).asString().c_str();
                double kp = command.get(2).asDouble();
                double ki = command.get(3).asDouble();
                double kd = command.get(4).asDouble();
                control_thr->set_pid(identif,kp,ki,kd);
                reply.addString("New pid parameters set.");
                fprintf(stderr,"New pid parameters set.\n");
            }
            return true;
        }
        else if (command.get(0).asString()=="reset_odometry")
        {
            if (control_thr)
            {
                control_thr->get_odometry_handler()->reset_odometry();
                reply.addString("Odometry reset done.");
            }
            return true;
        }
        reply.addString("Unknown command.");
        return true;
    }

    virtual bool close()
    {
        if (control_thr)
        {
            control_thr->stop();
            delete control_thr;
        }
        if (laser_thr)
        {
            laser_thr->stop();
            delete laser_thr;
        }
        if (compass_thr)
        {
            compass_thr->stop();
            delete compass_thr;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule()
    { 
        if (laser_thr)
        {
            laser_thr->printStats();
        }
        else
        {
            fprintf(stdout,"* Laser thread:\nnot running\n");
        }
        if (control_thr)
        {
            control_thr->printStats();
            control_thr->get_motor_handler()->updateControlMode();
            control_thr->get_odometry_handler()->printStats();
            control_thr->get_motor_handler()->printStats();
        }
        else
        {
            fprintf(stdout,"* Motor thread:\nnot running\n");
        }
        if (compass_thr)
        {
            compass_thr->printStats();
        }
        else
        {
            fprintf(stdout,"* Compass thread:\nnot running\n");
        }
        static int life_counter=0;
        fprintf(stdout,"* Life: %d\n", life_counter);
        life_counter++;
        fprintf(stdout,"\n");

        return true;
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);
    rf.setDefaultContext("iKart/conf");
    rf.setDefaultConfigFile("iKartCtrl.ini");

    if (rf.check("help"))
    {
        printf("\n");
        printf("Possible options: \n");
        printf("'rate <r>' sets the threads rate (default 20ms).\n");
        printf("'no_filter' disables command filtering.\n");
        printf("'no_motors' motor interface will not be opened.\n");
        printf("'no_laser' laser interface will not be opened.\n");
        printf("'fake_laser' a simulated laser sensor will be used instead of a real one (debug).\n");
        printf("'no_compass' inertial/compass ports will not be opened.\n");
        printf("'no_start' do not automatically enables pwm.\n");
        printf("'laser <filename>' starts the laser with the specified configuration file.\n");
        printf("'joystick_connect' tries to automatically connect to the joystickCtrl output.\n");
        printf("\n");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}
