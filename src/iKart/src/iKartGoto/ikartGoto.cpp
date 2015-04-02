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
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>

#include "ikartGoto.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//checks if a point is inside a polygon
int GotoThread::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}


bool GotoThread::compute_obstacle_avoidance()
{
    double min_distance = max_obstacle_distance;
    double min_angle    = 0.0;

    for (size_t i=0; i<1080; i++)
    {
        double curr_d     = laser_data.get_distance(i);
        double curr_angle = laser_data.get_angle(i);
        size_t angle_t = (size_t)(4.0 * frontal_blind_angle);
        if (i>=540-angle_t && i<=540+angle_t) continue; //skip frontalobstacles

        if (curr_d < min_distance)
        {
            min_distance = curr_d;
            min_angle = curr_angle;
        }
    }

    angle_f = min_angle;
    angle_t = angle_f+90.0;
    w_f     = (1-(min_distance/max_obstacle_distance))/2;
    w_t     = 0;
    return true;
}

bool GotoThread::check_obstacles_in_path()
{
    int laser_obstacles = 0;
    double goal_distance = 1000; //TO BE COMPLETED

    //compute the polygon
    double vertx[4];
    double verty[4];
    double theta = 0.0;
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double detection_distance = 1.5;
    if(enable_dynamic_max_distance)
        detection_distance = max_detection_distance * safety_coeff;
    else
        detection_distance = max_detection_distance;
    if (detection_distance<min_detection_distance) detection_distance=min_detection_distance;
    vertx[0]=(-robot_radius) * ctheta + detection_distance * (-stheta);
    verty[0]=(-robot_radius) * stheta + detection_distance * ctheta;
    vertx[1]=(+robot_radius) * ctheta + detection_distance * (-stheta);
    verty[1]=(+robot_radius) * stheta + detection_distance * ctheta;
    vertx[2]= +robot_radius * ctheta;
    verty[2]= +robot_radius * stheta;
    vertx[3]= -robot_radius * ctheta;
    verty[3]= -robot_radius * stheta;

    for (size_t i=0; i<1080; i++)
    {
        double d = laser_data.get_distance(i);
        if (d < robot_radius) 
        {
            laser_obstacles++;
            printf("obstacles on the platform\n");
            continue;
        }

        double px= laser_data.get_x(i);
        double py= laser_data.get_y(i);
        if (pnpoly(4,vertx,verty,px,py)>0)
        {
            double d = laser_data.get_distance(i);
            if (d < goal_distance)
            //if (laser_data.get_distance(i) < goal_distance)
            {
                laser_obstacles++;
                //printf("obstacles on the path\n");
                continue;
            }
            else
            {
                //printf("obstacles on the path, but goal is near\n");
                continue;
            }
        }
    }

    //prevent noise to be detected as an obtacle;
    if (laser_obstacles>=2)
    {
        printf("obstacles detected\n");
        return true;
    }

    //no obstacles found
    return false;
}

void GotoThread::run()
{
    mutex.wait();
    
    //data is formatted as follows: x, y, angle
    yarp::sig::Vector *loc = port_localization_input.read(false);
    if (loc) {localization_data = *loc; loc_timeout_counter=0;}
    else {loc_timeout_counter++; if (loc_timeout_counter>TIMEOUT_MAX) loc_timeout_counter=TIMEOUT_MAX;}
    if (loc_timeout_counter>=TIMEOUT_MAX)
    {
        if (status == MOVING)
        {
            printf ("stopping navigation because of localization timeouts!");
            status = ABORTED;
        }
    }

    yarp::sig::Vector *odm = port_odometry_input.read(false);
    if (odm) {odometry_data = *odm; odm_timeout_counter=0;}
    else {odm_timeout_counter++; if (odm_timeout_counter>TIMEOUT_MAX) odm_timeout_counter=TIMEOUT_MAX;}
    if (odm_timeout_counter>=TIMEOUT_MAX)
    {
        if (status == MOVING)
        {
            printf ("stopping navigation because of odometry timeouts!");
            status = ABORTED;
        }
    }

    yarp::os::Bottle *scan = port_laser_input.read(false);
    if (scan) {laser_data.set_cartesian_laser_data(scan); las_timeout_counter=0;}
    else {las_timeout_counter++; if (las_timeout_counter>TIMEOUT_MAX) las_timeout_counter=TIMEOUT_MAX;}

    //computes the control action
    control_out.zero();

    //gamma is the angle between the current ikart heading and the target heading
    double unwrapped_localization_angle = (localization_data[2]<0)?localization_data[2]+360:localization_data[2];
    double unwrapped_target_angle = (target_data[2]<0)?target_data[2]+360:target_data[2];      
    //double gamma  = localization_data[2]-target_data[2];
    double gamma  = unwrapped_localization_angle-unwrapped_target_angle;
    if      (gamma >  180) gamma -= 360;
    else if (gamma < -180) gamma += 360;

    //beta is the angle between the current ikart position and the target position
    double old_beta = atan2 (localization_data[1]-target_data[1],localization_data[0]-target_data[0])*180.0/M_PI;
    double beta = -atan2 (target_data[1]-localization_data[1],target_data[0]-localization_data[0])*180.0/M_PI;

    //distance is the distance between the current ikart position and the target position
    double distance = sqrt(pow(target_data[0]-localization_data[0],2) +  pow(target_data[1]-localization_data[1],2));

    //compute the control law
//    control_out[0] = -beta;
//    control_out[0] = -beta-localization_data[2]; //CHECKME
//    control_out[0] = -(beta-localization_data[2]); //CHECKME -180
//    control_out[0] = -(beta+localization_data[2]); //CHECKME -90
//    control_out[0] = +beta+localization_data[2]-90; //CHECKME -90
//    control_out[0] = +beta-localization_data[2]-90; //CHECKME -90
//    control_out[0] = -beta+localization_data[2]-90; //CHECKME -90
//    control_out[0] = -beta-localization_data[2]-90; //CHECKME -90
//    control_out[0] = -beta-localization_data[2]+90; //CHECKME -90
//    control_out[0] = -beta+localization_data[2]+90; //CHECKME -90
//    control_out[0] = +beta+localization_data[2]+90; //CHECKME -90
//    control_out[0] = +beta-localization_data[2]+90; //CHECKME -90
//    control_out[0] = -beta+localization_data[2]; //CHECKME -90
//    control_out[0] =  beta-localization_data[2]; //CHECKME -90
    double tmp1=  180-(old_beta-localization_data[2]);
    if (tmp1>360) 
        tmp1-=360;
    if (tmp1>180 && tmp1<360)
        tmp1 = tmp1-360;//ADDED LATER
    //printf ("%f \n", control[0]);
    control_out[1] =  k_lin_gain * distance;
    control_out[2] =  k_ang_gain * gamma;
    control_out[0] = tmp1;

    //control saturation
    //printf ("%f %f ", control_out[2], control_out[1]);
    if (control_out[2] > 0 )
    {
      if (control_out[2] > +max_ang_speed) control_out[2] = +max_ang_speed;
      if (control_out[2] < +min_ang_speed) control_out[2] = +min_ang_speed; 
    }
    else
    {
      if (control_out[2] < -max_ang_speed) control_out[2] = -max_ang_speed;
      if (control_out[2] > -min_ang_speed) control_out[2] = -min_ang_speed;
    }
   
    if (control_out[1] > 0 )
    {
      if (control_out[1] > +max_lin_speed) control_out[1] = +max_lin_speed;
      if (control_out[1] < +min_lin_speed) control_out[1] = +min_lin_speed; 
    }
    else
    {
      if (control_out[1] < -max_lin_speed) control_out[1] = -max_lin_speed;
      if (control_out[1] > -min_lin_speed) control_out[1] = -min_lin_speed;
    }
    //printf ("%f %f \n", control_out[2], control_out[1]);
    
    //check for large rotations: inhibit linear movement, to allow a rotation on place
    if (fabs(gamma)>25.0)
    {
        control_out[1] = 0;
    }

    //check for obstacles, always performed
    bool obstacles_in_path = false;
    if (las_timeout_counter < 300) 
    {
        obstacles_in_path = check_obstacles_in_path();
        compute_obstacle_avoidance();
        double correction = angle_f;
        if (correction<0) correction+=180;
        else correction-=180;
        double w_f_sat = w_f;
        if (w_f_sat>0.3) w_f_sat=0.3;
        double goal = control_out[0];
        double goal_corrected = goal * (1-w_f_sat) + correction * (w_f_sat);
        if (enable_obstacles_avoidance)
        {
            //direction is modified in proximity of the obstacles
            control_out[0] = goal_corrected;
            //speed is reduced in proximity of the obstacles
            double w_f_sat2 = w_f*2.2;
            if (w_f_sat2>0.85) w_f_sat2= speed_reduction_factor; //CHECK 0.85, put it config file
            control_out[1] = control_out[1] * (1.0-w_f_sat2);
        }
        else
        {
            control_out[0] = goal;
        }
        angle_g = goal_corrected;
    }
    else
    {
        if (status == MOVING)
        {
        }
    }

    double current_time = yarp::os::Time::now();
    double speed_ramp = (current_time-obstacle_removal_time)/2.0;

    switch (status.getStatusAsInt())
    {
        case MOVING:
            //Update the safety coefficient only if your are MOVING.
            //If you are WAITING_OBSTACLE, use the last valid safety_coeff until the 
            //obstacle has been removed.
            safety_coeff = control_out[1]/max_lin_speed;

            //compute the speed ramp after the removal of an obstacle
            speed_ramp = (speed_ramp > 1.0) ? 1.0 : speed_ramp;
            control_out[1] *= speed_ramp;
            control_out[2] *= speed_ramp;

            if (target_data.weak_angle)
            {
                //check if the goal has been reached in position but not in orientation
                if (fabs(distance) < goal_tolerance_lin)
                {
                    status=REACHED;
                    fprintf (stdout, "Goal reached!\n");
                }
            }
            else
            {
                 //check if the goal has been reached in both position and orientation
                if (fabs(distance) < goal_tolerance_lin && fabs(gamma) < goal_tolerance_ang) 
                {
                    status=REACHED;
                    fprintf (stdout, "Goal reached!\n");
                }

            }

            // check if you have to stop beacuse of an obstacle
            if  (enable_obstacles_emergency_stop && obstacles_in_path)
            {
                fprintf (stdout, "Obstacles detected, stopping \n");
                status=WAITING_OBSTACLE;
                obstacle_time = current_time;
                Bottle b;
                b.addString("Obstacles detected");
                Bottle tmp = port_speak_output.prepare();
                tmp.clear();
                tmp=b;
                port_speak_output.write();
            }
        break;

        case WAITING_OBSTACLE:
            if (!obstacles_in_path)
            {   
                if (fabs(current_time-obstacle_time)>1.0)
                {
                    fprintf (stdout, "Obstacles removed, thank you \n");
                    status=MOVING;
                    Bottle b;
                    b.addString("Obstacles removed, thank you");
                    Bottle tmp = port_speak_output.prepare();
                    tmp.clear();
                    tmp=b;
                    port_speak_output.write();
                    obstacle_removal_time = yarp::os::Time::now();
                }
            }
            else
            {
                if (fabs(current_time-obstacle_time)>max_obstacle_wating_time)
                {
                    fprintf (stdout, "failed to recover from obstacle, goal aborted \n");
                    status=ABORTED;
                }
            }
        break;

        case PAUSED:
            //check if pause is expired
            double current_time = yarp::os::Time::now();
            if (current_time - pause_start > pause_duration)
            {
                fprintf(stdout, "pause expired! resuming \n");
                status=MOVING;
            }
        break;
    }

    if (status != MOVING)
    {
       control_out[0]=control_out[1]=control_out[2] = 0.0;
    }

    if (enable_retreat && retreat_counter >0)
    {
        control_out[0]=180;
        control_out[1]=0.4;
        control_out[2]=0;
        retreat_counter--;
    }

    sendOutput();
    mutex.post();
}

void GotoThread::sendOutput()
{
    static yarp::os::Stamp stamp;
    stamp.update();
    //send the motors commands and the status to the yarp ports
    if (port_commands_output.getOutputCount()>0)
    {
        Bottle &b=port_commands_output.prepare();
        port_commands_output.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar commands
        b.addDouble(control_out[0]);    // angle in deg
        b.addDouble(control_out[1]);    // lin_vel in m/s
        b.addDouble(control_out[2]);    // ang_vel in deg/s
        port_commands_output.write();
    }

    if (port_status_output.getOutputCount()>0)
    {
        string string_out;
        string_out = status.getStatusAsString();
        Bottle &b=port_status_output.prepare();
        port_status_output.setEnvelope(stamp);
        b.clear();
        b.addString(string_out.c_str());
        port_status_output.write();
    }

    if(port_gui_output.getOutputCount()>0)
    {
        Bottle &b=port_gui_output.prepare();
        port_gui_output.setEnvelope(stamp);
        b.clear();
        b.addDouble(control_out[0]);
        b.addDouble(control_out[1]);
        b.addDouble(control_out[2]);
        b.addDouble(angle_f);
        b.addDouble(angle_t);
        b.addDouble(w_f);
        b.addDouble(w_t);
        b.addDouble(max_obstacle_distance);
        b.addDouble(angle_g);
        port_gui_output.write();
    }
}

void GotoThread::setNewAbsTarget(yarp::sig::Vector target)
{
    //data is formatted as follows: x, y, angle
    target_data.weak_angle=false;
    if (target.size()==2) 
    {
        //if the angle information is missing use as final orientation the direction in which the iKart has to move
        double beta = atan2 (localization_data[1]-target[1],localization_data[0]-target[0])*180.0/M_PI;
        double beta2 = beta-180;
        if (beta2>+180) beta2=360-beta2;
        if (beta2<-180) beta2=360+beta2;
        target.push_back(beta2);
        target_data.weak_angle=true;
    }
    target_data.target=target;
    status=MOVING;
    fprintf (stdout, "current pos: abs(%.3f %.3f %.2f)\n", localization_data[0], localization_data[1], localization_data[2]);
    fprintf (stdout, "received new target: abs(%.3f %.3f %.2f)\n", target_data[0], target_data[1], target_data[2]);
    retreat_counter = retreat_duration;
}

void GotoThread::setNewRelTarget(yarp::sig::Vector target)
{
    //data is formatted as follows: x, y, angle
    target_data.weak_angle=false;
    if (target.size()==2) 
    {
        target.push_back(0.0);
        target_data.weak_angle=true;
    }
    double a = localization_data[2]/180.0*M_PI;
    target_data[0]=target[1] * cos (a) - (-target[0]) * sin (a) + localization_data[0] ;
    target_data[1]=target[1] * sin (a) + (-target[0]) * cos (a) + localization_data[1] ;
    target_data[2]=-target[2] + localization_data[2];
    status=MOVING;
    fprintf (stdout, "received new target: abs(%.3f %.3f %.2f)\n", target_data[0], target_data[1], target_data[2]);
    retreat_counter = retreat_duration;
}

void GotoThread::pauseMovement(double secs)
{
    if (status == PAUSED)
    {
        fprintf (stdout, "already in pause!\n");
        return;
    }
    if (status != MOVING)
    {
        fprintf (stdout, "not moving!\n");
        return;
    }

    if (secs > 0)
    {
        fprintf (stdout, "asked to pause for %f \n", secs);
        pause_duration = secs;
    }
    else
    {
        fprintf (stdout, "asked to pause\n");
        pause_duration = 10000000;
    }
    status=PAUSED;
    pause_start = yarp::os::Time::now();
}

void GotoThread::resumeMovement()
{
    fprintf (stdout, "asked to resume movement\n");
    status=MOVING;
}

void GotoThread::stopMovement()
{
    fprintf (stdout, "asked to stop\n");
    status=IDLE;
}

string GotoThread::getNavigationStatus()
{
    return status.getStatusAsString();
}

void GotoThread::printStats()
{
    fprintf (stdout,"\n");
    fprintf (stdout,"* ikartGoto thread:\n");
    fprintf (stdout,"loc timeouts: %d\n",loc_timeout_counter);
    fprintf (stdout,"odm timeouts: %d\n",odm_timeout_counter);
    fprintf (stdout,"las timeouts: %d\n",las_timeout_counter);
    fprintf (stdout,"status: %s\n",status.getStatusAsString().c_str());
}
