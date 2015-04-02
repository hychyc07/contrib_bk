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
#include "iCub/safeReach/util.h"
#include "iCub/skinForceControl/util.h"
#include "iCub/skinForceControl/skinForceControlLib.h"

using namespace iCub::skinForceControl;

namespace iCub
{

namespace safeReach
{

// the last element of the enum (safeReachCommandSize) represents the total number of commands accepted by this module
enum safeReachCommand{ 
    reach,  stop,   track,
    set_kp, get_kp, set_kv, get_kv, 
    set_kf, get_kf, set_kd, get_kd, 
    set_ks, get_ks, set_fd, get_fd, 
    set_xd, get_xd, set_T, get_T, 
    set_q0, get_q0, set_qRest, get_qRest, 
    set_controller, get_controller, 
    help,   quit,   safeReachCommandSize
} ;

// the order of the command in this list MUST correspond to the order of the enum safeReachCommand
const std::string safeReachCommand_s[]  = {
    "reach",  "stop",   "track",
    "set kp", "get kp", "set kv", "get kv", 
    "set kf", "get kf", "set kd", "get kd", 
    "set ks", "get ks", "set fd", "get fd", 
    "set xd", "get xd", "set T", "get T", 
    "set q0", "get q0", "set qRest", "get qRest", 
    "set controller", "get controller", 
    "help", "quit"
};

// the order in safeReachCommand_desc must correspond to the order in safeReachCommand_s
const std::string safeReachCommand_desc[]  = {
    "start the reaching movement",
    "stop the motion",
    "track the target",
    "set kp", "get kp", "set kv", "get kv", 
    "set kf", "get kf", "set kd", "get kd", 
    "set ks", "get ks", "set fd", "get fd", 
    "set xd", "get xd", "set T", "get T", 
    "set q0", "get q0", "set qRest", "get qRest", 
    "set controller", "get controller",
    "get this list", 
	"quit module"};

static const int    DEFAULT_PERIOD          = 5;        // threa period in ms
static const int    PORT_TIMEOUT            = 10;       // timeout on ports in ms
static const float  DEFAULT_TRAJ_TIME_XD    = 4.0f;     // trajectory time (in sec)
static const int    N_JOINT                 = 10;       // number of joints of the kinematic chain

static const double MIN_POSITION_ERROR		= 5e-3;
static const double STABILITY_TIME_WINDOW_Q = 3.0;      // time window analysed to check if the controller is stable
static const double STABILITY_TIME_WINDOW_X = 8.0;      // time window analysed to check if the controller is stable
static const double STABILITY_TIME_WINDOW_F = 5.0;      // time window analysed to check if the controller is stable
static const double MIN_POS_ERR_IMPROVEMENT = 1e-3;     // minimum improvement in the position error to be taken into account
static const double X_STAB_THR              = 15e-3;    // stability threshold for x
static const double F_STAB_THR              = 2.0;      // stability threshold for f
static const double Q_STAB_THR              = 1.0;      // stability threshold for q

static const yarp::sig::Vector DEFAULT_KP = yarp::math::cat(1, 1, 1)*50.0;  // position proportional gains
static const yarp::sig::Vector DEFAULT_KV = yarp::math::cat(1, 1, 1)*50.0;  // position velocity gains
static const yarp::sig::Vector DEFAULT_KF = yarp::math::cat(1, 1, 1)*0.5;   // force proportional gains
static const yarp::sig::Vector DEFAULT_KD = yarp::math::cat(1, 1, 1)*0.02;  // force damping gains
static const double DEFAULT_KS = 1e4;                                       // environment contact stiffness

static const yarp::sig::Vector DEFAULT_FD  = yarp::math::cat(0, 0, 0);     // desired force
static const yarp::sig::Vector DEFAULT_XD_LEFT   = yarp::math::cat(-0.3   ,-0.15   , 0.0 );
static const yarp::sig::Vector DEFAULT_XD_RIGHT  = yarp::math::cat(-0.3   , 0.15   , 0.0 );
static const yarp::sig::Vector DEFAULT_Q0  = buildVector(N_JOINT, 0. , 0. , 0. , -45. ,  80. , 0. ,  70. , 0. , 0. , 0.);       // starting joint configuration
static const yarp::sig::Vector DEFAULT_Q_REST = buildVector(N_JOINT, 0. , 0. , 0. , -30. ,  30. , 0. ,  45. , 0. , 0. , 0.);    // rest joint configuration

const ControlLaw ctrlLaws[3] = {SAFE_REACH_RIGID_CTRL, SAFE_REACH_SPRING_CTRL, SAFE_REACH_ORTHO_CTRL };

}

} // end namespace



#endif
