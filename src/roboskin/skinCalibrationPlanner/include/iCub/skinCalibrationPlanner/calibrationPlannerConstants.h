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
#include "iCub/skinCalibrationPlanner/util.h"

namespace iCub
{

namespace skinCalibrationPlanner
{

// the last element of the enum (SkinForceCtrlCommandSize) represents the total number of commands accepted by this module
enum skinCalibrationPlannerCommand{ start,  stop,   help,   quit,   skinCalibrationPlannerCommandSize} ;

// the order of the command in this list MUST correspond to the order of the enum skinCalibrationPlannerCommand
const std::string skinCalibrationPlannerCommand_s[]  = {"start", "stop", "help", "quit"};

// the order in SkinForceCtrlCommand_desc must correspond to the order in SkinForceCtrlCommand_s
const std::string skinCalibrationPlannerCommand_desc[]  = {
    "start the calibration",
    "stop the calibration",
    "get this list", 
	"quit module"};

static const int    DEFAULT_PERIOD          = 5;        // planner period in ms
static const int    PORT_TIMEOUT            = 10;       // timeout on ports in ms
static const double TAXEL_POS_UPDATE_PERIOD = 10.0;     // period of update of taxel positions in sec
static const double DEFAULT_MAX_NEIGH_DIST  = 0.03;     // max distance between two neighbor taxels in m
static const double MIN_TOTAL_CONFIDENCE    = 6.0;      // min value of the total pose confidence to control the taxel position rather then the e.e.

static const double FEASIBILITY_THR         = 0.005;    // threshold for the feasibility index of a task
static const float  DEFAULT_TRAJ_TIME_XD    = 4.0f;     // trajectory time (in sec)
static const int    N_JOINT                 = 10;       // number of joints of the kinematic chain
static const double MIN_POSITION_ERROR		= 5e-3;		
static const double MIN_POS_ERR_IMPROVEMENT = 1e-3;     // minimum improvement in the position error to be taken into account

static const double FORCE_THRESHOLD         = 3.0;      // force norm threshold for considering the robot in contact
static const double FORCE_CONTACT_THRESHOLD = 5.0;      // force norm threshold for considering the robot in contact
static const double CONTACT_LOST_TIMEOUT    = 3.0;      // timeout for switching to ContactLost phase (in sec)
static const double STABILITY_TIME_WINDOW   = 8.0;      // time window analysed to check if the controller is stable
static const double SECOND_ATTEMPT_STABILITY_TIME_WINDOW = 5.0; // time window analysed to check if the controller is stable during the second attemp to reach a taxel
static const double CALIB_STABILITY_TIME_WINDOW = 2.0;  // time window analysed to check if the controller is stable
static const double X_STAB_THR              = 15e-3;    // stability threshold for x
static const double F_STAB_THR              = 1.0;      // stability threshold for f
static const double Q_STAB_THR              = 1.0;      // stability threshold for q

static const double MAX_ENV_CONTACT_DISTANCE  = 0.03;   // maximum distance of xEnv from the xEnv at startup

static const double SPIRAL_INITIAL_MOTION_STEP  = 0.01;    // increment used in the calib_spiral phase
static const double SPIRAL_STEP_INCREMENT       = 0.005;    // increment used in the calib_spiral phase
static const double SPIRAL_STABILITY_TIME_WINDOW = 16.0;     // time window analysed to check if the controller is stable

static const double	TAXEL_SELECTION_WEIGHT	= 0.5;		// weight The taxel pose confidence w.r.t the taxel visit count;
static const yarp::sig::Vector SCRUB_FORCE_CONTROL  = yarp::math::cat(0, 0, 5);     // force commanded when starting the scrub action
static const yarp::sig::Vector SCRUB_FORCE_CALIB  = yarp::math::cat(0, 0, 7);     // force commanded when starting the scrub action
static const yarp::sig::Vector MOTION_STEP  = yarp::math::cat(-0.03, 0, 0); // motion step used in the initial calibration phase
static const yarp::sig::Vector ELIGIBLE_TRIANGLES = yarp::math::cat(yarp::math::cat(5, 8, 9, 10), yarp::math::cat(11, 12, 13, 14));
static const unsigned int TAXELS_PER_TRIANGLE = 12;

}

} // end namespace



#endif
