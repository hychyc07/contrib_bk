/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef PLAN_CONST
#define PLAN_CONST

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "iCub/distanceImitation/util.h"

using namespace yarp::math;

namespace iCub
{

namespace distanceImitation
{

//calib start: activate float control so that the robot can be moved
//calib done: save current position as x0, move to xH, do a trial
//start: start the trials
//pause: finish current trial, then go home and stop
//home: move to xH and close all fingers but the index
// the last element of the enum (distanceImitationCommandSize) represents the total number of commands accepted by this module
enum distanceImitationCommand{ 
    calib_start,    calib_done, start,  start2, pause,  home,
    help,           quit,       distanceImitationCommandSize
} ;

// the order of the command in this list MUST correspond to the order of the enum distanceImitationCommand
const std::string distanceImitationCommand_s[]  = {
    "calib start",    "calib done", "start",  "start2", "pause",  "home", 
    "help", "quit"
};

// the order in distanceImitationCommand_desc must correspond to the order in distanceImitationCommand_s
const std::string distanceImitationCommand_desc[]  = {
    "activate float control so that the robot can be moved",
    "save current position as x0, move to xH, do a trial",
    "start the collaborative experiment",
    "start the non-collaborative experiment",
    "finish current trial, then go home and stop",
    "move to xH and close all fingers but the index",
    "get this list", 
	"quit module"};

static const int    DEFAULT_PERIOD          = 5;        // threa period in ms
static const int    PORT_TIMEOUT            = 10;       // timeout on ports in ms
static const int    N_JOINT                 = 19;       // number of joints of the kinematic chain
static const double MIN_TRAJ_TIME           = 1.0;      // minimum trajectory time
static const double MAX_TRAJ_TIME           = 2.0;      // maximum trajectory time
static const double CART_CTRL_TOL           = 1e-3;     // cartesian controller tollerance

static const double DEFAULT_T       = 0.0;      // T: time waited between trials (sec)
static const int    DEFAULT_K_MAX   = 10;       // k_max: upper bound of the interval in which k ranges
static const double DEFAULT_D_MIN   = 0.04;     // d_min: minimum distance between x1 and x2
static const double DEFAULT_STEP    = 0.008;    // step: step used for generating different distances
static const double DEFAULT_SPEED   = 0.1;      // speed: reference speed of the movement
static const int    DEFAULT_N       = 3;        // N: number of trial for each value of k
static const yarp::sig::Vector DEFAULT_GAZE_HOME    = cat(-10.0, 0.0, 0.4);
static const yarp::sig::Vector DEFAULT_Q_ARM_HOME   = cat(cat(cat(20, 0, 0,-25, 45), cat(0, 80, 0, 0, 0)), cat(cat(0, 10, 0, 150, 50), cat(0, 90, 130, 140)));
//static const yarp::sig::Vector DEFAULT_Q_ARM_HOME   = cat(cat(cat(0, 0, 0,-70, 90), cat(0, 75, 0, 0, 0)), cat(cat(0, 10, 0, 150, 50), cat(0, 90, 130, 140)));


}

} // end namespace



#endif
