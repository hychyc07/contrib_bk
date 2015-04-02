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

#include "controlThread.h"
#include "filters.h"

void ControlThread::apply_ratio_limiter (double& linear_speed, double& angular_speed)
{
    if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
    if (linear_speed>100)   linear_speed = 100;
    if (linear_speed<-100)  linear_speed = -100;
    if (angular_speed>100)  angular_speed = 100;
    if (angular_speed<-100) angular_speed = -100;

    double tot = fabs(linear_speed) + fabs(angular_speed);

    if (tot> 100)
    {
        //if lin_ang_ratio is negative, coeff will be 0 and
        //the adaptive limiter will be used (current ratio)
        double coeff = 0.0;
        if (lin_ang_ratio>0.0) coeff = (tot-100.0)/100.0;
        double angular_speed_A, angular_speed_B, linear_speed_A, linear_speed_B;

        angular_speed_A = angular_speed *(1-lin_ang_ratio);
        linear_speed_A  = linear_speed * lin_ang_ratio;

        double current_ratio = fabs(linear_speed/angular_speed);
        if (angular_speed>0) angular_speed_B =  100.0/(current_ratio+1.0);
        else                 angular_speed_B = -100.0/(current_ratio+1.0);

        linear_speed_B  =  100.0-fabs(angular_speed_B);
        //if (angular_speed>0) linear_speed_B  =  100.0-fabs(angular_speed_B);
        //else                 linear_speed_B  = -100.0+fabs(angular_speed_B);

        linear_speed  = linear_speed_A  *     (coeff) + linear_speed_B  * (1.0-coeff);
        angular_speed = angular_speed_A *     (coeff) + angular_speed_B * (1.0-coeff);

    }
}

void ControlThread::apply_ratio_limiter (double max, double& linear_speed, double& angular_speed)
{
    if (lin_ang_ratio<0.0)  lin_ang_ratio = 0.0;
    if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
    if (linear_speed  >  max*lin_ang_ratio) linear_speed  = max*lin_ang_ratio;
    if (linear_speed  < -max*lin_ang_ratio) linear_speed  = -max*lin_ang_ratio;
    if (angular_speed >  max*(1-lin_ang_ratio)) angular_speed = max*(1-lin_ang_ratio);
    if (angular_speed < -max*(1-lin_ang_ratio)) angular_speed = -max*(1-lin_ang_ratio);
}

void ControlThread::apply_input_filter (double& linear_speed, double& angular_speed, double& desired_direction)
{
    if (input_filter_enabled == 8)
    {
        angular_speed     = ikart_filters::lp_filter_8Hz(angular_speed,7);
        linear_speed      = ikart_filters::lp_filter_8Hz(linear_speed,8);
        desired_direction = ikart_filters::lp_filter_8Hz(desired_direction,9);
    }
    if (input_filter_enabled == 4)
    {
        angular_speed     = ikart_filters::lp_filter_4Hz(angular_speed,7);
        linear_speed      = ikart_filters::lp_filter_4Hz(linear_speed,8);
        desired_direction = ikart_filters::lp_filter_4Hz(desired_direction,9);
    }
    if (input_filter_enabled == 2)
    {
        angular_speed     = ikart_filters::lp_filter_2Hz(angular_speed,7);
        linear_speed      = ikart_filters::lp_filter_2Hz(linear_speed,8);
        desired_direction = ikart_filters::lp_filter_2Hz(desired_direction,9);
    }
    if (input_filter_enabled == 1)
    {
        angular_speed     = ikart_filters::lp_filter_1Hz(angular_speed,7);
        linear_speed      = ikart_filters::lp_filter_1Hz(linear_speed,8);
        desired_direction = ikart_filters::lp_filter_1Hz(desired_direction,9);
    }


}

void ControlThread::enable_debug(bool b)
{
    debug_enabled = b;
    if (b)
    {
        port_debug_direction.open((localName+"/debug/direction:o").c_str());
        port_debug_linear.open((localName+"/debug/linear:o").c_str());
        port_debug_angular.open((localName+"/debug/angular:o").c_str());
    }
    else
    {
        port_debug_linear.interrupt();
        port_debug_linear.close();
        port_debug_angular.interrupt();
        port_debug_angular.close();
        port_debug_direction.interrupt();
        port_debug_direction.close();
    }
}

void ControlThread::set_pid (string id, double kp, double ki, double kd)
{
    yarp::os::Bottle old_options;
    this->angular_speed_pid->getOptions(old_options);
    printf("Current configuration: %s\n",old_options.toString().c_str());
    
    // (Kp (10.0)) (Ki (0.0)) (Kf (0.0)) ... (satLim(-1000.0 1000.0)) (Ts 0.02)
    yarp::os::Bottle options;
    yarp::os::Bottle& bkp = options.addList();
    yarp::os::Bottle& bki = options.addList();
    yarp::os::Bottle& bkd = options.addList();
    bkp.addString("Kp");    yarp::os::Bottle& bkp2 = bkp.addList();    bkp2.addDouble(kp);
    bki.addString("Ki");    yarp::os::Bottle& bki2 = bki.addList();    bki2.addDouble(ki);
    bkd.addString("Kd");    yarp::os::Bottle& bkd2 = bkd.addList();    bkd2.addDouble(kd);
    printf("new configuration: %s\n",options.toString().c_str());

    this->angular_speed_pid->setOptions(options);
    yarp::sig::Vector tmp; tmp.resize(1); tmp.zero();
    this->angular_speed_pid->reset(tmp);
}

void ControlThread::apply_control_speed_pid(double& pidout_linear_speed,double& pidout_angular_speed, double& pidout_direction,
                           const double ref_linear_speed, const double ref_angular_speed, const double ref_desired_direction)
{
    double feedback_linear_speed = this->odometry_handler->ikart_vel_lin / MAX_LINEAR_VEL* 200;
    double feedback_angular_speed = this->odometry_handler->ikart_vel_theta / MAX_ANGULAR_VEL * 200;
    double feedback_desired_direction = this->odometry_handler->ikart_vel_heading;
    yarp::sig::Vector tmp;
    tmp = linear_speed_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
 //   pidout_linear_speed  = exec_pwm_gain * tmp[0];
    pidout_linear_speed  = 1.0 * tmp[0];
    tmp = angular_speed_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
   // pidout_angular_speed = exec_pwm_gain * tmp[0];
    pidout_angular_speed = 1.0 * tmp[0];
    tmp = direction_speed_pid->compute(yarp::sig::Vector(1,ref_desired_direction),yarp::sig::Vector(1,feedback_desired_direction));
    pidout_direction     = tmp[0];
        pidout_linear_speed=0;
        //pidout_angular_speed=0;
        pidout_direction=0;

    if (debug_enabled) // debug block
    {
        char buff [255];
        if (port_debug_linear.getOutputCount()>0)
        {
            Bottle &b1=port_debug_linear.prepare();
            b1.clear();
            sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_linear_speed,feedback_linear_speed,ref_linear_speed-feedback_linear_speed,pidout_linear_speed);
            b1.addString(buff);
            port_debug_linear.write();
        }

        if (port_debug_angular.getOutputCount()>0)
        {
            Bottle &b2=port_debug_angular.prepare();
            b2.clear();
            sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_angular_speed,feedback_angular_speed,ref_angular_speed-feedback_angular_speed,pidout_angular_speed);
            b2.addString(buff);
            port_debug_angular.write();
        }

        if (port_debug_direction.getOutputCount()>0)
        {
            Bottle &b3=port_debug_direction.prepare();
            b3.clear();
            sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_desired_direction,feedback_desired_direction,ref_desired_direction-feedback_desired_direction,pidout_direction);
            b3.addString(buff);
            port_debug_direction.write();
        }
    }
}

void ControlThread::apply_control_openloop_pid(double& pidout_linear_speed,double& pidout_angular_speed, double& pidout_direction,
                           const double ref_linear_speed,const double ref_angular_speed, const double ref_desired_direction)
{
    double feedback_linear_speed = this->odometry_handler->ikart_vel_lin / MAX_LINEAR_VEL* 100;
    double feedback_angular_speed = this->odometry_handler->ikart_vel_theta / MAX_ANGULAR_VEL * 100;
    double feedback_desired_direction = this->odometry_handler->ikart_vel_heading;
    yarp::sig::Vector tmp;
    tmp = linear_ol_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
    pidout_linear_speed  = exec_pwm_gain * tmp[0];
    tmp = angular_ol_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
    pidout_angular_speed = exec_pwm_gain * tmp[0];
    tmp = direction_ol_pid->compute(yarp::sig::Vector(1,ref_desired_direction),yarp::sig::Vector(1,feedback_desired_direction));
    pidout_direction     = tmp[0];
        pidout_linear_speed=0;
        //pidout_angular_speed=0;
        pidout_direction=0;

    if (debug_enabled) // debug block
    {
        char buff [255];
        Bottle &b1=port_debug_linear.prepare();
        b1.clear();
        sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_linear_speed,feedback_linear_speed,ref_linear_speed-feedback_linear_speed,pidout_linear_speed);
        b1.addString(buff);
        port_debug_linear.write();

        Bottle &b2=port_debug_angular.prepare();
        b2.clear();
        sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_angular_speed,feedback_angular_speed,ref_angular_speed-feedback_angular_speed,pidout_angular_speed);
        b2.addString(buff);
        port_debug_angular.write();
        
        Bottle &b3=port_debug_direction.prepare();
        b3.clear();
        sprintf(buff,"%+9.4f %+9.4f %+9.4f %+9.4f",ref_desired_direction,feedback_desired_direction,ref_desired_direction-feedback_desired_direction,pidout_direction);
        b3.addString(buff);
        port_debug_direction.write();
    }
}

void ControlThread::run()
{
    this->odometry_handler->compute();

    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;

    //input_linear_speed and input_angular speed ranges are: 0-100
    this->motor_handler->read_inputs(&input_linear_speed, &input_angular_speed, &input_desired_direction, &input_pwm_gain);
    apply_input_filter(input_linear_speed, input_angular_speed,input_desired_direction);
    apply_ratio_limiter(input_linear_speed, input_angular_speed);

    if (!lateral_movement_enabled) 
    {
        if (input_desired_direction>-90 && input_desired_direction <90) input_desired_direction = 0;
        else if (input_desired_direction <= -90) input_desired_direction = 180;
        else if (input_desired_direction >= +90) input_desired_direction = 180;
    }

    exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
    exec_desired_direction = input_desired_direction;

    //The controllers
    double MAX_VALUE = 0;
    if (ikart_control_type == IKART_CONTROL_OPENLOOP_NO_PID)
    {
        MAX_VALUE = 1250; // Maximum joint PWM
        exec_linear_speed  = input_linear_speed  / 100.0 * MAX_VALUE * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * MAX_VALUE * exec_pwm_gain;
        
        pidout_linear_speed  = exec_linear_speed;
        pidout_angular_speed = exec_angular_speed;
        pidout_direction     = exec_desired_direction;
        this->motor_handler->execute_openloop(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else if (ikart_control_type == IKART_CONTROL_SPEED_NO_PID)
    {
        MAX_VALUE = 200; // Maximum joint speed (deg/s)
        exec_linear_speed = input_linear_speed / 100.0 * MAX_VALUE * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * MAX_VALUE * exec_pwm_gain;

//#define PRINT_CURRENT_VEL
#ifdef  PRINT_CURRENT_VEL
        //printf("%+5.5f, %+5.5f, %+5.5f\n", input_angular_speed /100 * this->motor_handler->get_max_angular_vel(), this->odometry_handler->ikart_vel_theta, exec_angular_speed);
        printf("%+5.5f, %+5.5f, %+5.5f\n", input_linear_speed /100 * this->motor_handler->get_max_linear_vel(), this->odometry_handler->ikart_vel_lin, exec_linear_speed);
#endif

        pidout_linear_speed  = exec_linear_speed;
        pidout_angular_speed = exec_angular_speed;
        pidout_direction     = exec_desired_direction;
        this->motor_handler->execute_speed(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else if (ikart_control_type == IKART_CONTROL_OPENLOOP_PID)
    {
        MAX_VALUE = 1250; // Maximum joint PWM
        exec_linear_speed  = input_linear_speed  / 100.0 * MAX_VALUE * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * MAX_VALUE * exec_pwm_gain;
        
        apply_control_openloop_pid(pidout_linear_speed,pidout_angular_speed,pidout_direction,exec_linear_speed,exec_angular_speed,exec_desired_direction);
        this->motor_handler->execute_speed(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else if (ikart_control_type == IKART_CONTROL_SPEED_PID)
    {
        MAX_VALUE = 200; // Maximum joint speed (deg/s)
        exec_linear_speed = input_linear_speed / 100.0 * MAX_VALUE * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * MAX_VALUE * exec_pwm_gain;
        
        apply_control_speed_pid(pidout_linear_speed,pidout_angular_speed,pidout_direction,exec_linear_speed,exec_angular_speed,exec_desired_direction);
        
        pidout_angular_speed += exec_angular_speed;
        this->motor_handler->execute_speed(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else
    {
        printf ("ERROR! unknown control mode \n");
        exec_linear_speed = 0;
        exec_angular_speed = 0;
        exec_pwm_gain = 0;
        exec_desired_direction = 0;
        this->motor_handler->execute_none();
    }
}

void ControlThread::printStats()
{
    fprintf (stdout,"* Control thread:\n");
    fprintf (stdout,"Input command: %+5.0f %+5.0f %+5.0f  %+5.0f      ",input_linear_speed, input_angular_speed, input_desired_direction, input_pwm_gain);
    fprintf (stdout, "***** %+5.2f %+5.2f\n", input_linear_speed/100.0*this->get_motor_handler()->get_max_linear_vel(), input_angular_speed/100.0*this->get_motor_handler()->get_max_angular_vel());
}

bool ControlThread::set_control_type (string s)
{
    if      (s == "none")            ikart_control_type = IKART_CONTROL_NONE;
    else if (s == "speed_no_pid")    ikart_control_type = IKART_CONTROL_SPEED_NO_PID;
    else if (s == "openloop_no_pid") ikart_control_type = IKART_CONTROL_OPENLOOP_NO_PID;
    else if (s == "speed_pid")       ikart_control_type = IKART_CONTROL_SPEED_PID;
    else if (s == "openloop_pid")    ikart_control_type = IKART_CONTROL_OPENLOOP_PID;
    else
    {
        fprintf(stderr,"Error: unknown type of control required: %s. Closing...\n",s.c_str());
        return false;
    }
    fprintf(stderr,"Control type set to: %s\n",s.c_str());
    return true;
}
