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
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>

#include "status.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define TIMEOUT_MAX 300
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

struct target_type
{
    yarp::sig::Vector target;
    bool              weak_angle;

    target_type() {weak_angle=false; target.resize(3,0.0);}
    double& operator[] (const int& i) { return target[i]; }
};

class laser_type
{
    double laser_x   [1080];
    double laser_y   [1080];
    double distances [1080];
    double angles    [1080];
    
    public:
    laser_type ()
    {
        unsigned int i = 0;
        for (i=0;i<1080; i++) laser_x[i]=0.0;
        for (i=0;i<1080; i++) laser_y[i]=0.0;
        for (i=0;i<1080; i++) distances[i]=0.0;
        for (i=0;i<1080; i++) angles[i]=0.0;
    }

    void set_cartesian_laser_data (const yarp::os::Bottle* laser_map)
    {
        if (laser_map==0) return;
        for (unsigned int i=0; i<1080; i++)
        {
            Bottle* elem = laser_map->get(i).asList();
            if (elem ==0) printf ("ERROR\n");
            laser_x[i] = elem->get(0).asDouble();
            laser_y[i] = elem->get(1).asDouble();
            distances[i]=sqrt(laser_x[i]*laser_x[i]+laser_y[i]*laser_y[i]);
            angles[i] = atan2(double(laser_x[i]),double(laser_y[i]))*RAD2DEG;
        }
    }

    inline const double& get_distance(int i) { return distances[i]; }
    inline const double& get_angle (int i) { return angles[i]; }
    inline const double& get_x (int i) { return laser_x[i]; }
    inline const double& get_y (int i) { return laser_y[i]; }
};

class GotoThread: public yarp::os::RateThread
{
    private:
    void sendOutput();

    public:
    bool   enable_retreat;
    double goal_tolerance_lin;  //m 
    double goal_tolerance_ang;  //deg

    public:
    //configuration parameters
    double k_ang_gain;
    double k_lin_gain;
    double max_lin_speed;       //m/s
    double max_ang_speed;       //deg/s
    double min_lin_speed;       //m/s
    double min_ang_speed;       //deg/s
    double robot_radius;        //m
    int    retreat_duration; 

    int    loc_timeout_counter;
    int    odm_timeout_counter;
    int    las_timeout_counter;

    //semaphore
    Semaphore mutex;

    protected:
    //pause info
    double pause_start;
    double pause_duration;

    //ports
    BufferedPort<yarp::sig::Vector> port_odometry_input;
    BufferedPort<yarp::sig::Vector> port_localization_input;
    BufferedPort<yarp::sig::Vector> port_target_input;
    BufferedPort<yarp::os::Bottle>  port_laser_input;
    BufferedPort<yarp::os::Bottle>  port_commands_output;
    BufferedPort<yarp::os::Bottle>  port_status_output;
    BufferedPort<yarp::os::Bottle>  port_speak_output;
    BufferedPort<yarp::os::Bottle>  port_gui_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   odometry_data;
    target_type         target_data;
    laser_type          laser_data;
    yarp::sig::Vector   control_out;
    status_type         status;
    int                 retreat_counter;

    //obstacles_emergency_stop block
    public:
    bool                enable_obstacles_emergency_stop;
    bool                enable_dynamic_max_distance;
    double              free_distance[1080];
    double              obstacle_time;
    double              max_obstacle_wating_time;
    double              safety_coeff;
    double              max_detection_distance;
    double              min_detection_distance;
    double              obstacle_removal_time;

    //obstacle avoidance block
    public:
    bool                enable_obstacles_avoidance;
    double              max_obstacle_distance;
    double              frontal_blind_angle;
    double              speed_reduction_factor;
    double              angle_f;
    double              angle_t;
    double              angle_g;
    double              w_f;
    double              w_t;
    double              w_g;

    public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        status = IDLE;
        loc_timeout_counter = TIMEOUT_MAX;
        odm_timeout_counter = TIMEOUT_MAX;
        las_timeout_counter = TIMEOUT_MAX;
        localization_data.resize(3,0.0);
        retreat_counter = 0;
        safety_coeff = 1.0;
        enable_obstacles_emergency_stop = false;
        enable_obstacles_avoidance      = false;
        enable_dynamic_max_distance     = false;
        enable_retreat                  = false;
        retreat_duration                = 300;
        control_out.resize(3,0.0);
        pause_start = 0;
        pause_duration = 0;
        goal_tolerance_lin = 0.05;
        goal_tolerance_ang = 0.6;
        max_obstacle_wating_time = 60.0;
        max_obstacle_distance = 0.8;
        frontal_blind_angle = 25.0;
        speed_reduction_factor = 0.70;
        max_detection_distance = 1.5;
        min_detection_distance = 0.4;
        obstacle_removal_time = 0.0;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        k_ang_gain = 0.05;
        k_lin_gain = 0.1;
        max_lin_speed = 0.9;  //m/s
        max_ang_speed = 10.0; //deg/s
        min_lin_speed = 0.0;  //m/s
        min_ang_speed = 0.0; //deg/s
        robot_radius = 0.30;  //m
        printf ("Using following paramters:\n %s\n", rf.toString().c_str());
        if (rf.check("ang_speed_gain"))     {k_ang_gain = rf.find("ang_speed_gain").asDouble();}
        if (rf.check("lin_speed_gain"))     {k_lin_gain = rf.find("lin_speed_gain").asDouble();}
        if (rf.check("max_lin_speed"))      {max_lin_speed = rf.find("max_lin_speed").asDouble();}
        if (rf.check("max_ang_speed"))      {max_ang_speed = rf.find("max_ang_speed").asDouble();}
        if (rf.check("min_lin_speed"))      {min_lin_speed = rf.find("min_lin_speed").asDouble();}
        if (rf.check("min_ang_speed"))      {min_ang_speed = rf.find("min_ang_speed").asDouble();}
        if (rf.check("robot_radius"))       {robot_radius = rf.find("robot_radius").asDouble();}
        if (rf.check("goal_tolerance_lin")) {goal_tolerance_lin = rf.find("goal_tolerance_lin").asDouble();}
        if (rf.check("goal_tolerance_ang")) {goal_tolerance_ang = rf.find("goal_tolerance_ang").asDouble();}

        Bottle btmp;
        btmp = rf.findGroup("RETREAT_OPTION");
        if (btmp.check("enable_retreat",Value(0)).asInt()==1)
            enable_retreat = true;
        retreat_duration = btmp.check("retreat_duration",Value(300)).asInt();

        btmp = rf.findGroup("OBSTACLES_EMERGENCY_STOP");
        if (btmp.check("enable_obstacles_emergency_stop",Value(0)).asInt()==1)
            enable_obstacles_emergency_stop = true;
        if (btmp.check("enable_dynamic_max_distance",Value(0)).asInt()==1)
            enable_dynamic_max_distance = true;
        max_obstacle_wating_time = btmp.check("max_wating_time",Value(60.0)).asDouble();
        max_detection_distance   = btmp.check("max_detection_distance",Value(1.5)).asDouble();
        min_detection_distance   = btmp.check("min_detection_distance",Value(0.4)).asDouble();
        
        btmp = rf.findGroup("OBSTACLES_AVOIDANCE");
        if (btmp.check("enable_obstacles_avoidance",Value(0)).asInt()==1)
            enable_obstacles_avoidance = true;
        if (btmp.check("frontal_blind_angle"))
            frontal_blind_angle = btmp.check("frontal_blind_angle",Value(25.0)).asDouble();
        if (btmp.check("speed_reduction_factor"))
            speed_reduction_factor = btmp.check("speed_reduction_factor",Value(0.70)).asDouble();

        enable_retreat = false;
        retreat_duration = 300;

        //open module ports
        string localName = "/ikartGoto";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_target_input.open((localName+"/target:i").c_str());
        port_laser_input.open((localName+"/laser_map:i").c_str());
        port_commands_output.open((localName+"/control:o").c_str());
        port_status_output.open((localName+"/status:o").c_str());
        port_odometry_input.open((localName+"/odometry:i").c_str());
        port_speak_output.open((localName+"/speak:o").c_str());
        port_gui_output.open((localName+"/gui:o").c_str());

        //automatic connections for debug
        yarp::os::Network::connect("/ikart/laser:o","/laserScannerGui/laser:i");
        yarp::os::Network::connect("/ikartGoto/gui:o","/laserScannerGui/nav_display:i");

        //automatic port connections
        /*bool b = false;
        b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the localization port!"); return false;}
        b = Network::connect((localName+"/commands:o").c_str(),"/ikart/control:i", "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the output command port!"); return false;}
        b = Network::connect("/ikart/laser:o",(localName+"/laser:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the laser port!"); }*/

        //compute the free distance
        for (int i=0; i<1080; i++)
        {
            free_distance[i] = fabs(robot_radius / sin(((double(i)/1080.0)*270.0-135.0)*DEG2RAD));
            if (free_distance[i]>2.0) free_distance[i] = 2.0;
        }

        return true;
    }

    virtual void run();

    void setNewAbsTarget(yarp::sig::Vector target);
    void setNewRelTarget(yarp::sig::Vector target);
    void stopMovement();
    void pauseMovement (double secs);
    void resumeMovement();
    string getNavigationStatus();

    virtual void threadRelease()
    {    
        port_localization_input.interrupt();
        port_localization_input.close();
        port_target_input.interrupt();
        port_target_input.close();
        port_laser_input.interrupt();
        port_laser_input.close();
        port_commands_output.interrupt();
        port_commands_output.close();
        port_status_output.interrupt();
        port_status_output.close();
        port_odometry_input.interrupt();
        port_odometry_input.close();
        port_speak_output.interrupt();
        port_speak_output.close();
        port_gui_output.interrupt();
        port_gui_output.close();
    }

    void printStats();
    bool check_obstacles_in_path();
    bool compute_obstacle_avoidance();
    
    private:
    int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);

};

#endif
