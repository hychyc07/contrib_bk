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

#ifndef ODOMETRY_H
#define ODOMETRY_H

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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class Odometry
{
private:
    Property iKartCtrl_options;
    yarp::os::Semaphore   mutex;
    yarp::os::Stamp       timeStamp;

    //encoder variables
    double              encA_offset;
    double              encB_offset;
    double              encC_offset;

    double              encA;
    double              encB;
    double              encC;

    //measured motor velocity
    double              velA;
    double              velB;
    double              velC;

    //estimated motor velocity
    double              velA_est;
    double              velB_est;
    double              velC_est;
    iCub::ctrl::AWLinEstimator      *encvel_estimator;

    //robot geometry
    double              geom_r;
    double              geom_L;

    //thread period
    double              period;

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

public:
    //estimated cartesian velocity in the fixed odometry reference frame (world)
    double              odom_vel_x;
    double              odom_vel_y;
    double              odom_vel_heading;
    double              odom_vel_theta;

    //estimated cartesian velocity in the ikart relative reference frame
    double              ikart_vel_lin;
    double              ikart_vel_x;
    double              ikart_vel_y;
    double              ikart_vel_heading;
    double              ikart_vel_theta;

    //estimated odometer 
    double              traveled_distance;
    double              traveled_angle;

    //estimated cartesian pos in the fixed odometry reference frame
    double              odom_x;
    double              odom_y;
    double              odom_theta;

private:
    ResourceFinder                  &rf;
    PolyDriver                      *control_board_driver;
    BufferedPort<Bottle>            port_odometry;
    BufferedPort<Bottle>            port_odometer;
    BufferedPort<Bottle>            port_ikart_vels;

    string                          localName;
    IEncoders                       *ienc;

public:
    Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver); 
    ~Odometry();
    bool reset_odometry();
    bool open();
    void compute();
    void printStats();
    void close();

};

#endif
