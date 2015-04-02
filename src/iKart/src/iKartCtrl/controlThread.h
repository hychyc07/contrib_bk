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

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

#include "odometry.h"
#include "motors.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define MAX_LINEAR_VEL  0.4  // maximum linear  velocity (m/s)
#define MAX_ANGULAR_VEL 24.0 // maximum angular velocity (deg/s)

enum
{
    IKART_CONTROL_NONE = 0,
    IKART_CONTROL_OPENLOOP_NO_PID = 1,
    IKART_CONTROL_OPENLOOP_PID = 2,
    IKART_CONTROL_SPEED_NO_PID = 3,
    IKART_CONTROL_SPEED_PID = 4
};

class ControlThread : public yarp::os::RateThread
{
private:
    Property            iKartCtrl_options;

    double              thread_period;
    int                 ikart_control_type;
    int                 thread_timeout_counter;

    //the current command
    double input_linear_speed;
    double input_angular_speed;
    double input_desired_direction;
    double input_pwm_gain;

    //movement control variables (internally computed)
    double              exec_linear_speed;
    double              exec_angular_speed;
    double              exec_desired_direction;
    double              exec_pwm_gain;

    //ikart pids
    iCub::ctrl::parallelPID*    linear_speed_pid;
    iCub::ctrl::parallelPID*    angular_speed_pid;
    iCub::ctrl::parallelPID*    direction_speed_pid;
    iCub::ctrl::parallelPID*    linear_ol_pid;
    iCub::ctrl::parallelPID*    angular_ol_pid;
    iCub::ctrl::parallelPID*    direction_ol_pid;

    //controller parameters
    double              lin_ang_ratio;
    bool                both_lin_ang_enabled;
    int                 input_filter_enabled;
    bool                lateral_movement_enabled;
    bool                debug_enabled;

protected:
    ResourceFinder            &rf;
    PolyDriver                *control_board_driver;
    BufferedPort<Bottle>      port_movement_control;
    BufferedPort<Bottle>      port_auxiliary_control;
    BufferedPort<Bottle>      port_joystick_control;

    BufferedPort<Bottle>      port_debug_direction;
    BufferedPort<Bottle>      port_debug_linear;
    BufferedPort<Bottle>      port_debug_angular;

    Odometry*                 odometry_handler;
    MotorControl*             motor_handler;

    string                    remoteName;
    string                    localName;

public:
    Odometry* const     get_odometry_handler() {return odometry_handler;}
    MotorControl* const get_motor_handler()    {return motor_handler;}
    void                enable_debug(bool b);

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        control_board_driver     = 0;
        thread_timeout_counter   = 0;
        ikart_control_type       = IKART_CONTROL_NONE;

        input_filter_enabled     = iKartCtrl_options.findGroup("GENERAL").check("input_filter_enabled",Value(0),"input filter frequency (1/2/4/8Hz, 0 = disabled)").asInt();
        lateral_movement_enabled = (iKartCtrl_options.findGroup("GENERAL").check("lateral_movement_enabled",Value(1),"1 = lateral moevements enabled, 0 = disabled").asInt()>0);
        lin_ang_ratio            = iKartCtrl_options.findGroup("GENERAL").check("linear_angular_ratio",Value(0.7),"ratio (<1.0) between the maximum linear speed and the maximum angular speed.").asDouble();

        debug_enabled            = false;
        both_lin_ang_enabled     = true;
        thread_period            = _period;

        input_linear_speed       = 0;
        input_angular_speed      = 0;
        input_desired_direction  = 0;
        input_pwm_gain           = 0;
        linear_speed_pid         = 0;
        angular_speed_pid        = 0;
        direction_speed_pid      = 0;
        linear_ol_pid            = 0;
        angular_ol_pid           = 0;
        direction_ol_pid         = 0;
        remoteName               = iKartCtrl_options.find("remote").asString();
        localName                = iKartCtrl_options.find("local").asString();
    }

    virtual bool threadInit()
    {
        string control_type = iKartCtrl_options.findGroup("GENERAL").check("control_mode",Value("none"),"type of control for the wheels").asString().c_str();
        set_control_type (control_type);

        // open the control board driver
        printf("\nOpening the motors interface...\n");
        int trials=0;
        double start_time = yarp::os::Time::now();
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote",remoteName.c_str());
        control_board_options.put("local",localName.c_str());

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (control_board_driver)
            {
                delete control_board_driver;
                control_board_driver=0;
            }

            //creates the new device driver
            control_board_driver=new PolyDriver(control_board_options);
            bool connected =control_board_driver->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (10s) is expired
            if (current_time-start_time > 10.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                if (control_board_driver)
                {
                    delete control_board_driver;
                    control_board_driver=0;
                }
                return false;
            }

            yarp::os::Time::delay(0.5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        //create the odometry and the motor handlers
        odometry_handler = new Odometry ((int)(thread_period),rf,iKartCtrl_options,control_board_driver);
        motor_handler = new MotorControl((int)(thread_period),rf,iKartCtrl_options,control_board_driver);
        odometry_handler->open();
        motor_handler->open();

        printf ("%s",iKartCtrl_options.toString().c_str());
        //create the pid controllers
        if (!iKartCtrl_options.check("HEADING_SPEED_PID"))
        {
            fprintf(stderr,"\nError reading from .ini file, section PID\n");
            return false;
        }
        if (!iKartCtrl_options.check("LINEAR_SPEED_PID"))
        {
            fprintf(stderr,"\nError reading from .ini file, section PID\n");
            return false;
        }
        if (!iKartCtrl_options.check("ANGULAR_SPEED_PID"))
        {
            fprintf(stderr,"\nError reading from .ini file, section PID\n");
            return false;
        }
        yarp::sig::Vector kp[3],ki[3],kd[3];
        yarp::sig::Vector wp[3],wi[3],wd[3];
        yarp::sig::Vector N[3];
        yarp::sig::Vector Tt[3];
        yarp::sig::Matrix sat[3];
        for (int i=0; i<3; i++)
        {
            kp[i].resize(1); ki[i].resize(1); kd[i].resize(1);
            kp[i]=ki[i]=kd[i]=0.0;
            wp[i].resize(1); wi[i].resize(1); wd[i].resize(1);
            wp[i]=wi[i]=wd[i]=1.0;
            N[i].resize(1); Tt[i].resize(1); sat[i].resize(1,2);
            N[i]=10;
            Tt[i]=1;
        }

        kp[0]=iKartCtrl_options.findGroup("LINEAR_SPEED_PID").check("kp",Value(0),"kp gain").asDouble();
        kd[0]=iKartCtrl_options.findGroup("LINEAR_SPEED_PID").check("kd",Value(0),"kd gain").asDouble();
        ki[0]=iKartCtrl_options.findGroup("LINEAR_SPEED_PID").check("ki",Value(0),"ki gain").asDouble();
        sat[0](0,0)=iKartCtrl_options.findGroup("LINEAR_SPEED_PID").check("min",Value(0),"min").asDouble();
        sat[0](0,1)=iKartCtrl_options.findGroup("LINEAR_SPEED_PID").check("max",Value(0),"max").asDouble();

        kp[1]=iKartCtrl_options.findGroup("ANGULAR_SPEED_PID").check("kp",Value(0),"kp gain").asDouble();
        kd[1]=iKartCtrl_options.findGroup("ANGULAR_SPEED_PID").check("kd",Value(0),"kd gain").asDouble();
        ki[1]=iKartCtrl_options.findGroup("ANGULAR_SPEED_PID").check("ki",Value(0),"ki gain").asDouble();
        sat[1](0,0)=iKartCtrl_options.findGroup("ANGULAR_SPEED_PID").check("min",Value(0),"min").asDouble();
        sat[1](0,1)=iKartCtrl_options.findGroup("ANGULAR_SPEED_PID").check("max",Value(0),"max").asDouble();

        kp[2]=iKartCtrl_options.findGroup("HEADING_SPEED_PID").check("kp",Value(0),"kp gain").asDouble();
        kd[2]=iKartCtrl_options.findGroup("HEADING_SPEED_PID").check("kd",Value(0),"kd gain").asDouble();
        ki[2]=iKartCtrl_options.findGroup("HEADING_SPEED_PID").check("ki",Value(0),"ki gain").asDouble();
        sat[2](0,0)=iKartCtrl_options.findGroup("HEADING_SPEED_PID").check("min",Value(0),"min").asDouble();
        sat[2](0,1)=iKartCtrl_options.findGroup("HEADING_SPEED_PID").check("max",Value(0),"max").asDouble();

        linear_speed_pid    = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[0],ki[0],kd[0],wp[0],wi[0],wd[0],N[0],Tt[0],sat[0]);
        angular_speed_pid   = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[1],ki[1],kd[1],wp[1],wi[1],wd[1],N[1],Tt[1],sat[1]);
        direction_speed_pid = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[2],ki[2],kd[2],wp[2],wi[2],wd[2],N[2],Tt[2],sat[2]);

        linear_ol_pid    = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[0],ki[0],kd[0],wp[0],wi[0],wd[0],N[0],Tt[0],sat[0]);
        angular_ol_pid   = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[1],ki[1],kd[1],wp[1],wi[1],wd[1],N[1],Tt[1],sat[1]);
        direction_ol_pid = new iCub::ctrl::parallelPID(thread_period/1000.0,kp[2],ki[2],kd[2],wp[2],wi[2],wd[2],N[2],Tt[2],sat[2]);

        //debug ports
        if (debug_enabled)
        {
            port_debug_direction.open((localName+"/debug/direction:o").c_str());
            port_debug_linear.open((localName+"/debug/linear:o").c_str());
            port_debug_angular.open((localName+"/debug/angular:o").c_str());
        }

        //start the motors
        if (!rf.check("no_start"))
        {
            motor_handler->turn_on_control();
        }

        if (rf.check("speed_pid"))            this->set_control_type("speed_pid");
        else if (rf.check("speed_no_pid"))    this->set_control_type("speed_no_pid");
        else if (rf.check("openloop_pid"))    this->set_control_type("openloop_pid");
        else if (rf.check("openloop_no_pid")) this->set_control_type("openloop_no_pid");
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Control thread started successfully\n");
        else
            printf("Control thread did not start\n");
    }

    virtual void run();
    bool set_control_type (string s);
    void printStats();
    void set_pid (string id, double kp, double ki, double kd);
    void apply_ratio_limiter (double max, double& linear_speed, double& angular_speed);
    void apply_ratio_limiter (double& linear_speed, double& angular_speed);
    void apply_input_filter  (double& linear_speed, double& angular_speed, double& desired_direction);
    void set_input_filter    (int b) {input_filter_enabled=b;}
    
    void apply_control_openloop_pid(double& pidout_linear_speed,double& pidout_angular_speed, double& pidout_direction, const double ref_linear_speed,const double ref_angular_speed, const double ref_desired_direction);
    void apply_control_speed_pid(double& pidout_linear_speed,double& pidout_angular_speed, double& pidout_direction, const double ref_linear_speed,const double ref_angular_speed, const double ref_desired_direction);

    virtual void threadRelease()
    {
        if (odometry_handler)  {delete odometry_handler; odometry_handler=0;}
        if (motor_handler)     {delete motor_handler; motor_handler=0;}

        if (linear_speed_pid)    {delete linear_speed_pid;  linear_speed_pid=0;}
        if (angular_speed_pid)   {delete angular_speed_pid; angular_speed_pid=0;}
        if (direction_speed_pid) {delete direction_speed_pid; direction_speed_pid=0;}
        if (linear_ol_pid)    {delete linear_ol_pid;  linear_ol_pid=0;}
        if (angular_ol_pid)   {delete angular_ol_pid; angular_ol_pid=0;}
        if (direction_ol_pid) {delete direction_ol_pid; direction_ol_pid=0;}

        if (debug_enabled)
        {
            port_debug_linear.interrupt();
            port_debug_linear.close();
            port_debug_angular.interrupt();
            port_debug_angular.close();
            port_debug_direction.interrupt();
            port_debug_direction.close();
        }
    }
};

#endif
