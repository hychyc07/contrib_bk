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

#ifndef CTRL_CONST
#define CTRL_CONST

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "iCub/skinForceControl/util.h"

namespace iCub
{

namespace skinForceControl
{

// the last element of the enum (SkinForceCtrlCommandSize) represents the total number of commands accepted by this module
enum SkinForceCtrlCommand{ gaze_on,    gaze_off,   help,	quit,   SkinForceCtrlCommandSize} ;

// the order of the command in this list MUST correspond to the order of the enum SkinForceCtrlCommand
const std::string SkinForceCtrlCommand_s[]  = { "gaze on", "gaze off", "help", "quit"};

// the order in SkinForceCtrlCommand_desc must correspond to the order in SkinForceCtrlCommand_s
const std::string SkinForceCtrlCommand_desc[]  = {
    "activate gaze control",
    "deactivate gaze control",
    "get this list", 
	"quit module"};

enum Action { NO_ACTION, FLOAT, LINE_SCRUB, DYN_LINE_SCRUB, ADAP_LSCRUB, CIRCLE, ACTION_SIZE };
const std::string Action_desc[ACTION_SIZE] = { "NO_ACTION", "FLOAT", "LINE SCRUB", "DYN_LINE_SCRUB", "ADAPTIVE LINE SCRUB", "CIRCLE" };

enum CommandMode { SIMULATION, REAL };              // SIMULATION: it doesn't send torques to the motors, but just to the output port

static const int    DEFAULT_CTRL_PERIOD = 5;        // default control thread period (ms)
static const int    DEFAULT_GAZE_PERIOD = 300;      // default gaze control period (ms)
#ifdef _DEBUG
static const double PORT_TIMEOUT        = 2000;     // timeout for port reading (sec)  (increased to tolerate network glitches)
#else
static const double PORT_TIMEOUT        = 2;        // timeout for port reading (sec)  (increased to tolerate network glitches)
#endif
//static const double SAFE_FORCE_THR      = 3.0;        // force norm threshold (N)
static const double FEASIBILITY_THR         = 0.005;    // threshold for the feasibility index of a task
static const float  DEFAULT_FC_F            = 1.0f;     // contact force low pass filter cut frequency (in Hertz)
static const float  DEFAULT_FC_F_SMOOTH     = 0.2f;     // contact force low pass filter cut frequency (in Hertz)
static const float  DEFAULT_FC_FD           = 0.99f;    // reference force low pass filter cut frequency (in Hertz)
static const float  DEFAULT_FC_REF          = 0.5f;     // commanded torque low pass filter cut frequency (in Hertz)
static const float  DEFAULT_FC_COP          = 1.0f;     // contact point low pass filter cut frequency (in Hertz)
static const float  DEFAULT_TRAJ_TIME_XD    = 5.0f;     // trajectory time for xd (in sec)
static const float  DEFAULT_TRAJ_TIME_CTRL_PNT_D = 6.0f;// trajectory time for desired control point (in sec)
static const float  DEFAULT_TRAJ_TIME_QD    = 5.0f;     // trajectory time for qd (in sec)
static const int    N_JOINT                 = 10;       // number of joints of the kinematic chain
static const double DEFAULT_PINV_DAMP       = 4e-2;     // pseudo-inverse damping factor
static const bool   DEFAULT_CONTROL_FORCE_NORM = true;  // if true then only the force norm is controlled

static const double FORCE_THRESHOLD     = 5.0;      // threshold on contact force
static const double MAX_POS_ERROR       = 0.30;     // saturation limit for the position error
static const double XD_STAB_THR         = 0.001;    // stability threshold for xd
static const double FD_STAB_THR         = 0.01;     // stability threshold for fd
static const double DEFAULT_ETA         = 1e-5;     // learning rate for adaptive line scrub action
static const yarp::sig::Vector SCRUB_FORCE      = buildVector(3, 0., 0., 5.);   // force commanded when starting the scrub action
static const yarp::sig::Vector SCRUB_LINE_PATH  = buildVector(3, .03, 0., 0.);  // relative path commanded when starting the scrub action
static const double CIRCLE_RAY          = 0.05;     // ray of circle path for circle action
static const double CIRCLE_VEL          = 0.05;     // speed for circle action

// DEFAULT JOINT BOUND THRESHOLDS AND GAINS
static const yarp::sig::Vector DEFAULT_DQ_MAX(N_JOINT, 30.0);
static const yarp::sig::Vector DEFAULT_KJ(N_JOINT, 0.3);
//static const yarp::sig::Vector DEFAULT_KJ(N_JOINT, 0.7);
static const yarp::sig::Vector DEFAULT_ACTIVATION_THR(N_JOINT, 10);
static const yarp::sig::Vector DEFAULT_DEACTIVATION_THR(N_JOINT, 10);

// DEFAULT CONTROLLER GAINS
static const yarp::sig::Vector DEFAULT_JPOS_KP(N_JOINT, 60.0);
static const yarp::sig::Vector DEFAULT_JPOS_KD(N_JOINT, 40.0);
//static const yarp::sig::Vector DEFAULT_JPOS_KP = buildVector(N_JOINT, 0.    , 0.    , 0.    , 0.05   , 0.05   , 0.05   , 0.03  , 0.003  , 0.  , 0.);
static const yarp::sig::Vector DEFAULT_JPOS_KI(N_JOINT, 0.0);
//static const yarp::sig::Vector DEFAULT_JPOS_KD = buildVector(N_JOINT, 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.003 , 0.  , 0.);

static const yarp::sig::Vector DEFAULT_POS_KP(3, 100.0);
static const yarp::sig::Vector DEFAULT_POS_KI(3, 0.0);
static const yarp::sig::Vector DEFAULT_POS_KD(3, 0.01);

static const yarp::sig::Vector DEFAULT_FORCE_KP(6, 0.5);
static const yarp::sig::Vector DEFAULT_FORCE_KI(6, 0.05);
static const yarp::sig::Vector DEFAULT_FORCE_KD = buildVector(N_JOINT, 0.01  , 0.01  , 0.01  , 0.01  , 0.01  , 0.01  , 0.01  , 0.003 , 0.  , 0.);

static const yarp::sig::Vector DEFAULT_PARAL_KP(3, 150.0);
static const yarp::sig::Vector DEFAULT_PARAL_KV(3, 40.0);
static const yarp::sig::Vector DEFAULT_PARAL_KI(3, 0.02);
static const yarp::sig::Vector DEFAULT_PARAL_KD = buildVector(N_JOINT, 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.02  , 0.003 , 0.  , 0.);
static const yarp::sig::Vector DEFAULT_PARAL_KF(3, 0.5);

static const yarp::sig::Vector DEFAULT_DPARAL_KP(3, 150.0);
static const yarp::sig::Vector DEFAULT_DPARAL_KV(3, 50.0);
static const yarp::sig::Vector DEFAULT_DPARAL_KP2(N_JOINT, 1.0);

static const yarp::sig::Vector DEFAULT_CONTACT_KP(3, 50.0);
static const yarp::sig::Vector DEFAULT_CONTACT_KV(3, 250.0);
static const yarp::sig::Vector DEFAULT_CONTACT_KI(3, 0.02);
static const yarp::sig::Vector DEFAULT_CONTACT_KD = buildVector(N_JOINT, 0.0 , 0.0 , 0.0 , 0.02 , 0.02 , 0.02 , 0.02 , 0.003 , 0.0 , 0.0 );
static const yarp::sig::Vector DEFAULT_CONTACT_KF(3, 0.5);
static const yarp::sig::Vector DEFAULT_CONTACT_KC = buildVector(N_JOINT, 0.0 , 0.0 , 0.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 0.0 , 0.0 );
static const double            DEFAULT_CONTACT_TRAJ_TIME = 3.0;
static const double            DEFAULT_CONTACT_FORCE_PRIORITY = 10.0;

static const yarp::sig::Vector DEFAULT_PRESS_KP(3, 1.0);

static const yarp::sig::Vector DEFAULT_REACH_KP(3, 50.0);
static const yarp::sig::Vector DEFAULT_REACH_KV(3, 50.0);
static const yarp::sig::Vector DEFAULT_REACH_KP_REST(N_JOINT, 40.0);
static const yarp::sig::Vector DEFAULT_REACH_KD(N_JOINT, 20.0);

// DEFAULT CONTROLLER SET POINTS
static const yarp::sig::Vector DEFAULT_QD        = buildVector(N_JOINT, 0. , 0. , 0. , -30. ,  30. , 0. ,  45. , 0. , 0. , 0.);
static const yarp::sig::Vector DEFAULT_XD_LEFT   = buildVector(7, -0.298   ,-0.211   , 0.029   , 0.0     , 0.992   ,-0.127   , 2.958   );
static const yarp::sig::Vector DEFAULT_XD_RIGHT  = buildVector(7, -0.298   , 0.211   , 0.029   , 0.0     , 0.992   ,-0.127   , 2.958   );
static const yarp::sig::Vector DEFAULT_FD(6, 0.0);

// FIRMWARE TORQUE PID GAINS
//static const int FIRMWARE_KP_NEW_BLACK[]    = { 2000     , 2000      , 3600      , 3200 };
static const int FIRMWARE_KP_NEW_BLACK[]    = { 1000     , 1000      , 1800      , 2000 , 8000 };
static const int FIRMWARE_KI_NEW_BLACK[]    = { 2        , 2         , 1         , 2    , 30     };
static const int FIRMWARE_KD_NEW_BLACK[]    = { 0        , 0         , 0         , 0    , 0     };
static const int FIRMWARE_SHIFT_NEW_BLACK[] = { 16       , 16        , 16        , 16   , 16    };
//static const int FIRMWARE_FC_NEW_BLACK[]    = { 800      , 1500      , 2500      , 3000 };
static const int FIRMWARE_FC_NEW_BLACK[]    = { 800      , 1000      , 1000      , 1000 , 3000  };
static const int FIRMWARE_MAX_NEW_BLACK[]   = { 100      , 100       , 50        , 60   , 500  };

static const int FIRMWARE_KP_NEW_WHITE[]    = { 3000     , 5500      , 3200      , 4000 , 12000 };
static const int FIRMWARE_KI_NEW_WHITE[]    = { 1        , 1         , 4         , 4    , 3     };
static const int FIRMWARE_KD_NEW_WHITE[]    = { 0        , 0         , 0         , 0    , 0     };
static const int FIRMWARE_SHIFT_NEW_WHITE[] = { 16       , 16        , 16        , 16   , 16    };
static const int FIRMWARE_FC_NEW_WHITE[]    = { 325      , 242       , 1198      , 838  , 3000  };
static const int FIRMWARE_MAX_NEW_WHITE[]   = { 200      , 200       , 100       , 100  , 500   };

static const int FIRMWARE_KP_OLD[]      = { 512      , 512       , 1920      , 1920 , 3200  };
static const int FIRMWARE_KI_OLD[]      = { 0        , 0         , 0         , 0    , 0     };
static const int FIRMWARE_KD_OLD[]      = { 0        , 0         , 0         , 0    , 0     };
static const int FIRMWARE_SHIFT_OLD[]   = { 16       , 16        , 16        , 16   , 16    };
static const int FIRMWARE_FC_OLD[]      = { 3000     , 3000      , 3000      , 3000 , 3000  };
static const int FIRMWARE_MAX_OLD[]     = { 1333     , 1333      , 1333      , 1333 , 1333  };

static const double GRAV_ACC = 9.81;

}

} // end namespace



#endif
