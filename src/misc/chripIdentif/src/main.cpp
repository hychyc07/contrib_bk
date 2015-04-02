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
#include "command.h"

YARP_DECLARE_DEVICES(icubmod)

#define PRINT_STATUS_PER    0.200     // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

//==============================================================
class CtrlThread: public RateThread
{

private:
    enum
    {
        CONTROL_NONE = 0,
        CONTROL_OPENLOOP = 1,
        CONTROL_SPEED = 2,
        CONTROL_POSITION = 3,
    };

    double    command;
    int       cycle;
    int       times;
    double    period;
    
    int                 control_type;
    double              wdt_timeout;
    double              measure_position;
    double              measure_speed;
    double              measure_pwm;
    int                 joint;
    CommandClass        *commandProc;
    FILE *              pFile;

protected:
    ResourceFinder      &rf;
    PolyDriver          *control_board_driver;
    PolyDriver          *debug_driver;
    BufferedPort<Bottle>            port_movement_control;

    string remoteName;
    string localName;

    IPidControl       *ipid;
    IPositionControl  *ipos;
    IVelocityControl  *ivel;
    IEncoders         *ienc;
    IAmplifierControl *iamp;
    IOpenLoopControl  *iopl;
    IControlMode      *icmd;
    IDebugInterface   *idbg;

    double t0;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
    {
        //control_type = CONTROL_SPEED;
        //control_type = CONTROL_OPENLOOP;
        //control_type = CONTROL_NONE;
        control_type = CONTROL_POSITION;
        wdt_timeout = 0.100;
        command=0;
        joint=6;
        //commandProc = new CommandClass(0.5,-0.5,1.0);
        if (control_type == CONTROL_SPEED)
        {
            //commandProc = new CommandClass(CommandClass::COMMAND_SQUARE,0,2,5.0); //slow speed
            commandProc = new CommandClass(CommandClass::COMMAND_SQUARE,0,80,4.0);  //fast speed
        }
        else if (control_type == CONTROL_POSITION)
        {
            commandProc = new CommandClass(CommandClass::COMMAND_CHIRP,25,5,2.0);  //fast speed
        }
    }

    void set_control_type(int type)
    {
        control_type = type;
    }

    int get_control_type()
    {
        return control_type;
    }

    virtual bool threadInit()
    {

        pFile = fopen ("command_log3.txt","w");

        control_board_driver=new PolyDriver;
        bool ok = true;

        // open the control board driver
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote",remoteName.c_str());
        control_board_options.put("local",localName.c_str());
        if (!control_board_driver->open(control_board_options))
        {
            fprintf(stderr,"ERROR: cannot open control board driver...\n");
            delete control_board_driver;    
            return false;
        }
        // open the interfaces for the control boards
        ok = true;
        ok = ok & control_board_driver->view(ivel);
        ok = ok & control_board_driver->view(ienc);
        ok = ok & control_board_driver->view(iopl);
        ok = ok & control_board_driver->view(ipid);
        ok = ok & control_board_driver->view(iamp);
        ok = ok & control_board_driver->view(icmd);
        ok = ok & control_board_driver->view(ipos);
        if(!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
            //return false;
        }

        // open the debug driver
        Property debug_options;
        localName+="/debug/";
        debug_options.put("local", localName.c_str());
        debug_options.put("device", "debugInterfaceClient");
        debug_options.put("remote", remoteName.c_str());
        debug_driver = new PolyDriver(debug_options);
        if(debug_driver->isValid() == false)
        {
            fprintf(stderr,"ERROR: cannot open debugInterfaceClient...\n");
            delete debug_driver;    
            return false;
        }
        // open the interfaces for the control boards
        ok = true;
        ok = ok & debug_driver->view(idbg);
        if(!ok)
        {
            fprintf(stderr,"ERROR: idbg interface has not been viewed\nreturning...\n");
            return false;
        }

        // open ports
        port_movement_control.open((localName+"/control:i").c_str());

        /*
        //sets the control mode to the joints
        if (control_type == CONTROL_OPENLOOP)
        {
            fprintf(stdout,"Using openloop control mode\n");
            icmd->setOpenLoopMode(joint);
            iopl->setOutput(joint,0);
        }
        if (control_type == CONTROL_SPEED)
        {
            fprintf(stdout,"Using speed control mode\n");
            icmd->setVelocityMode(joint);
        }
        */
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            cout<<"Thread started successfully"<<endl;
        else
            cout<<"Thread did not start"<<endl;

        t0=Time::now();
    }

    virtual void run()
    {
        double t=Time::now();
        static double t1=Time::now();
        static int count = 0;
        int times; 

        commandProc->update(&command, &cycle, &times, &period);
        if (control_type == CONTROL_OPENLOOP)
        {
            iopl->setOutput(joint,command);
        }
        else if    (control_type == CONTROL_POSITION)
        {
            ipid->setReference(joint,command);
        }
        else if    (control_type == CONTROL_SPEED)
        {
            ivel->velocityMove(joint,command);
        }
        else if (control_type == CONTROL_NONE)
        {
            //iopl->setOutput(joint,0);
        }

        //log stuff
        ienc->getEncoder(joint,&measure_position);
        ienc->getEncoderSpeed(joint,&measure_speed);
        ipid->getOutput(joint,&measure_pwm);
       
        //fprintf (stdout,
        if (cycle != 0)
        {
            fprintf (pFile,
                            "%4d %3d  %3d  %+8.3f %+8.2f %+8.2f %+8.2f %+8.2f %+8.2f \n",
                            count,
                            cycle,
                            times,
                            period,
                            t-t1,
                            command,
                            measure_position,
                            measure_speed,
                            measure_pwm);
        }
        //t1=t;
        count++;

        printStatus();
    }

    virtual void threadRelease()
    {    
        delete control_board_driver;

        port_movement_control.interrupt();
        port_movement_control.close();
    }

    void turn_off_control()
    {
        set_control_type (CONTROL_NONE);
    }

    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
            //fprintf (stdout,"alive, time: %f\n",t-t0);
            //fprintf (stdout,"cycle: %d time: %+.3f command: %+.2f measure %+.2f error: %+.2f\n",cycle, t-t0, command, measure, command-measure);
            //fprintf (stdout,"\n");
            t0=t;
        }
    }
};


//==============================================================
class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    CtrlModule() { }

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
        ctrlName=rf.check("ctrlName",Value("commandGenerator")).asString();
 
        //robotName=rf.check("robot",Value("icubV2")).asString();
        //remoteName=slash+robotName+"/right_arm";

        robotName=rf.check("robot",Value("icubV2")).asString();
        remoteName=slash+robotName+"/head";


        localName=slash+ctrlName;//+"/local/";

        thr=new CtrlThread(10,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}
