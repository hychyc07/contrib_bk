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

#include "odometry.h"

bool Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encA_offset);
    ienc->getEncoder(1,&encB_offset);
    ienc->getEncoder(2,&encC_offset);
    odom_x=0;
    odom_y=0;
    encvel_estimator->reset();
    fprintf(stderr,"Odometry reset done\n");
    return true;
}

void Odometry::printStats()
{
    mutex.wait();
    //fprintf (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    fprintf (stdout,"* Odometry Thread:\n");
    fprintf (stdout,"enc1:%+9.1f enc2:%+9.1f enc3:%+9.1f ******** env1:%+9.3f env2:%+9.3f env3:%+9.3f\n",
    enc[0]*57, enc[1]*57, enc[2]*57, encv[0]*57, encv[1]*57, encv[2]*57);
    
    fprintf (stdout,"ivlx:%+9.3f ivly:%+9.3f                ******** ovlx:%+9.3f ovly:%+9.3f ovlt:%+9.3f ******** x: %+5.3f y: %+5.3f t: %+5.3f\n",
    ikart_vel_x, ikart_vel_y, odom_vel_x, odom_vel_y, ikart_vel_theta,  odom_x, odom_y,odom_theta );
    mutex.post();
}

void Odometry::close()
{    
    port_odometry.interrupt();
    port_odometry.close();
    port_odometer.interrupt();
    port_odometer.close();
    port_ikart_vels.interrupt();
    port_ikart_vels.close();
}

Odometry::~Odometry()
{
    close();
}

Odometry::Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver):rf(_rf)
{
    iKartCtrl_options = options;
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_theta=0;
    odom_vel_x=0;
    odom_vel_y=0;
    ikart_vel_x=0;
    ikart_vel_y=0;

    ikart_vel_lin=0;
    ikart_vel_theta=0;
    odom_vel_heading=0;
    ikart_vel_heading=0;
    traveled_distance=0;
    traveled_angle=0;
    geom_r = 62.5/1000.0;     //m
    geom_L = 297.16/1000.0;   //m
    encvel_estimator =new iCub::ctrl::AWLinEstimator(3,1.0);
    enc.resize(3);
    encv.resize(3);
    localName = iKartCtrl_options.find("local").asString();
}

bool Odometry::open()
{
    // open the control board driver
    printf("\nOpening the motors interface...\n");

    Property control_board_options("(device remote_controlboard)");
    if (!control_board_driver)
    {
        fprintf(stderr,"ERROR: control board driver not ready!\n");
            return false;
    }
    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ienc);
    if(!ok)
    {
        fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
        //return false;
    }
    // open control input ports
    port_odometry.open((localName+"/odometry:o").c_str());
    port_odometer.open((localName+"/odometer:o").c_str());
    port_ikart_vels.open((localName+"/velocity:o").c_str());

    //reset odometry
    reset_odometry();

    return true;
}

void Odometry::compute()
{
    mutex.wait();

    //read the encoders (deg)
    ienc->getEncoder(0,&encA);
    ienc->getEncoder(1,&encB);
    ienc->getEncoder(2,&encC);
        
    //read the speeds (deg/s)
    ienc->getEncoderSpeed(0,&velA);
    ienc->getEncoderSpeed(1,&velB);
    ienc->getEncoderSpeed(2,&velC);
        
    //remove the offset and convert in radians
    enc[0]= -(encA - encA_offset) * 0.0174532925; 
    enc[1]= -(encB - encB_offset) * 0.0174532925;
    enc[2]= -(encC - encC_offset) * 0.0174532925;
       
    //estimate the speeds
    iCub::ctrl::AWPolyElement el;
    el.data=enc;
    el.time=Time::now();
    encv= encvel_estimator->estimate(el);

    // -------------------------------------------------------------------------------------
    // The following formulas are adapted from:
    // "A New Odometry System to reduce asymmetric Errors for Omnidirectional Mobile Robots"
    // -------------------------------------------------------------------------------------

    //compute the orientation. odom_theta is expressed in radians
    odom_theta = geom_r*(enc[0]+enc[1]+enc[2])/(3*geom_L);

    //build the kinematics matrix
    yarp::sig::Matrix kin;
    kin.resize(3,3);
    kin.zero();
    kin(0,0) = -sqrt(3.0)/2.0;
    kin(0,1) = 0.5;
    kin(0,2) = geom_L;
    kin(1,0) = sqrt(3.0)/2.0;
    kin(1,1) = 0.5;
    kin(1,2) = geom_L;
    kin(2,0) = 0;
    kin(2,1) = -1.0;
    kin(2,2) = geom_L;
    kin      = kin/geom_r;
    yarp::sig::Matrix ikin = luinv(kin);

    //build the rotation matrix
    yarp::sig::Matrix m1;
    m1.resize(3,3);
    m1.zero();
    m1(0,0) = cos (odom_theta);
    m1(0,1) = -sin (odom_theta);
    m1(1,0) = sin (odom_theta);
    m1(1,1) = cos (odom_theta);
    m1(2,2) = 1;

    yarp::sig::Matrix m2;
    m2.resize(3,3);
    m2.zero();
    m2(0,0) = cos (0.0);
    m2(0,1) = -sin (0.0);
    m2(1,0) = sin (0.0);
    m2(1,1) = cos (0.0);
    m2(2,2) = 1;

    yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
    yarp::sig::Vector ikart_cart_vels; //velocities expressed in the ikart reference frame
    odom_cart_vels  = m1*ikin*encv;
    ikart_cart_vels = m2*ikin*encv;

    ikart_vel_x     = ikart_cart_vels[1];
    ikart_vel_y     = ikart_cart_vels[0];
    ikart_vel_theta = ikart_cart_vels[2];
    ikart_vel_lin   = sqrt(odom_vel_x*odom_vel_x + odom_vel_y*odom_vel_y);
    
    odom_vel_x      = odom_cart_vels[1];
    odom_vel_y      = odom_cart_vels[0];
    odom_vel_theta  = odom_cart_vels[2];
  
    //these are not currently used
    if (ikart_vel_lin<0.001)
    {
        odom_vel_heading  = 0;
        ikart_vel_heading = 0;
    }
    else
    {
        odom_vel_heading  = atan2(odom_vel_x,odom_vel_y)*57.2957795;
        ikart_vel_heading = atan2(ikart_vel_x,ikart_vel_y)*57.2957795;
    }

    //the integration step
    odom_x=odom_x + (odom_vel_x * period/1000.0);
    odom_y=odom_y + (odom_vel_y * period/1000.0);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(ikart_vel_lin   * period/1000.0);
    traveled_angle    = traveled_angle    + fabs(ikart_vel_theta * period/1000.0);

    /* [ -(3^(1/2)*r)/3, (3^(1/2)*r)/3,        0]
        [            r/3,           r/3, -(2*r)/3]
        [        r/(3*L),       r/(3*L),  r/(3*L)]*/

    /*odom_x = -1.73205080*geom_r/3 * encA + 1.73205080*geom_r/3 * encB;
    odom_y = geom_r/3 * encA +  geom_r/3 * encB - (2*geom_r)/3 * encC;*/
        
    /*
    odom_x = geom_r/(3* 0.86602)*
                (
                (co3p-co3m)*encA + 
                (-cos(odom_theta)-co3p)*encB + 
                (cos(odom_theta)+co3m)*encC
                );
    odom_y = geom_r/(3* 0.86602)*
                (
                (si3m+si3p)*encA + 
                (-sin(odom_theta)-si3p)*encB + 
                (sin(odom_theta)-si3m)*encC
                );
    */

    //convert from radians back to degrees
    odom_theta       *= 57.2957795;
    ikart_vel_theta  *= 57.2957795;
    odom_vel_theta   *= 57.2957795;
    traveled_angle   *= 57.2957795;

    mutex.post();

    timeStamp.update();
    if (port_odometry.getOutputCount()>0)
    {
        port_odometry.setEnvelope(timeStamp);
        Bottle &b=port_odometry.prepare();
        b.clear();
        b.addDouble(odom_x);
        b.addDouble(odom_y);
        b.addDouble(odom_theta);
        b.addDouble(odom_vel_x);
        b.addDouble(odom_vel_y);
        b.addDouble(odom_vel_theta);
        port_odometry.write();
    }

    if (port_odometer.getOutputCount()>0)
    {
        port_odometer.setEnvelope(timeStamp);
        Bottle &t=port_odometer.prepare();
        t.clear();
        t.addDouble(traveled_distance);
        t.addDouble(traveled_angle);
        port_odometer.write();
    }

    if (port_ikart_vels.getOutputCount()>0)
    {
        port_ikart_vels.setEnvelope(timeStamp);
        Bottle &v=port_ikart_vels.prepare();
        v.clear();
        v.addDouble(ikart_vel_x);
        v.addDouble(ikart_vel_y);
        v.addDouble(ikart_vel_theta);
        port_ikart_vels.write();
    }
}
