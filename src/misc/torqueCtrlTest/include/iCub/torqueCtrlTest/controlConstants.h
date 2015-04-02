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

namespace iCub
{

namespace torqueCtrlTest
{

// the last element of the enum (SFC_COMMAND_COUNT) represents the total number of commands accepted by this module
enum TorqueCtrlTestCommand{ 
	no_ctrl,            set_ctrl,           get_ctrl,
    set_body_part,      get_body_part,
    set_kp,             get_kp,
    set_kd,             get_kd,
    set_ki,             get_ki,
    set_taod,           get_taod,           get_tao,
    set_pwm,            get_pwm,
    set_alpha,          get_alpha,
    sim_on,             sim_off,
    set_joint,          get_joint,
    set_ktao,           get_ktao,
    set_kbemf,          get_kbemf,
    set_kstic,          get_kstic,
    set_kdither,        get_kdither,
    set_wdither,        get_wdither,
    set_kcp,            get_kcp,
    set_kcn,            get_kcn,
    set_est_wind,       get_est_wind,
    set_est_thr,        get_est_thr,
    set_qd,             get_qd,
    set_traj_time,      get_traj_time,
    set_impedance,      get_impedance,
    reset_pid,
    help,				quit,
    SFC_COMMAND_COUNT} ;

// the order of the command in this list MUST correspond to the order of the enum SkinForceCtrlCommand
const std::string TorqueCtrlTestCommand_s[]  = {
    "stop",                 "set ctrl",             "get ctrl",
    "set body part",        "get body part",
    "set kp",               "get kp",
    "set kd",               "get kd",
    "set ki",               "get ki",
    "set taod",             "get taod",             "get tao",
    "set pwm",              "get pwm",
    "set alpha",            "get alpha",
    "sim on",               "sim off",
    "set jnt",              "get jnt",
    "set ktao",             "get ktao",           
    "set kbemf",            "get kbemf",
    "set kstic",            "get kstic",
    "set kdith",            "get kdith",
    "set wdith",            "get wdith",
    "set kcp",              "get kcp",
    "set kcn",              "get kcn",
    "set est wind",         "get est wind",
    "set est thr",          "get est thr",
    "set qd",               "get qd",
    "set tt",               "get tt",
    "set impedance",        "get impedance",
    "reset",
    "help",                 "quit"};

// the order in SkinForceCtrlCommand_desc must correspond to the order in SkinForceCtrlCommand_s
const std::string TorqueCtrlTestCommand_desc[]  = {
    "stop the controller",
	"set the control law (0 none, 1 pid, 2 open loop, 3 feedforward, 4 pos, 5 dither, 6 impedance)",
    "get the current control law",
    "set body part (2 TORSO, 3 LEFT_ARM, 4 RIGHT_ARM, 5 LEFT_LEG, 6 RIGHT_LEG)",
    "get body part",
	"set the proportional gains (double)",
    "get the proportional gains (double)",
	"set the derivative gains (double)",
    "get the derivative gains (double)",
    "set the integral gains (double)",
    "get the integral gains (double)",
    "set the desired joint torques",
    "get the desired joint torques",
    "get the current joint torques",
    "set the desired pwm",
    "get the desired pwm",
	"set low pass filter cut freq for pwm and measured torque (Hz)",
    "get low pass filter cut freq for pwm and measured torque (Hz)",
    "activate the simulation mode (the computed torques are not commanded to the motors)",
    "deactivate the simulation mode (the computed torques are not commanded to the motors)",
    "set the joint to control",
    "get the joint to control",
    "set_ktao",             
    "get_ktao",           
    "set_kbemf",            
    "get_kbemf",
    "set k_stic and k_coulomb",
    "get k_stic and k_coulomb",
    "set kdith",
    "get kdith",
    "set wdith",
    "get wdith",
    "set kcp",
    "get kcp",
    "set kcn",
    "get kcn",
    "set_est_wind",
    "get_vel_wind",
    "set_est_thr",
    "get_est_thr",
    "set desired joint pos",
    "get desired joint pos",
    "set trajectory time",
    "get trajectory time",
    "set impedance parameters (0 desired mass, 1 real mass, 2 damping, 3 stiffness)",
    "get impedance parameters (0 desired mass, 1 real mass, 2 damping, 3 stiffness)",
    "reset the torque pid integral error",
    "get this list", 
	"quit the module"};

enum ControlThreadStatus { STATUS_OK=0, STATUS_DISCONNECTED };

enum ControlMode { NO_CONTROL, PID_CTRL, OPEN_CTRL, FEEDFORWARD_CTRL, POS_CTRL, DITHER_CTRL, IMPEDANCE_CTRL, CONTROL_MODE_SIZE };
const std::string ControlMode_desc[] = { "NO_CONTROL", "PID_CTRL", "OPEN_CTRL", "FEEDFORWARD_CTRL", "POS_CTRL", "DITHER_CTRL", "IMPEDANCE_CTRL"};

enum CommandMode { SIMULATION, REAL };              // SIMULATION: it doesn't send torques to the motors, but just to the output port

static const float  DEFAULT_ALPHA = 10.0f;             // contact force low pass filter intensity (in [0, 1])
static const int    DEFAULT_JOINT_ID = 3;              // id of the joint to control
static const int    DEFAULT_SAT_LIM_UP = 200;
static const int    DEFAULT_SAT_LIM_DOWN = -200;
static const int    DEFAULT_VEL_EST_WIND = 7;
static const double DEFAULT_VEL_EST_THR = 1.0;
static const double DEFAULT_TRAJ_TIME = 1.5;

// DEFAULT CONTROLLER GAINS
static const double DEFAULT_PID_KP[]   = { 5.0 };
static const double DEFAULT_PID_KI[]   = { 000.0 }; // integral is dangerous because it makes stiction increase and when it overcomes it, it goes overboard
static const double DEFAULT_PID_KD[]   = { 000.0};
static const int DEFAULT_PID_KP_SIZE   = sizeof(DEFAULT_PID_KP)/sizeof(double);
static const int DEFAULT_PID_KI_SIZE   = sizeof(DEFAULT_PID_KI)/sizeof(double);
static const int DEFAULT_PID_KD_SIZE   = sizeof(DEFAULT_PID_KD)/sizeof(double);

// GAIN OF MOTOR PLANT
static const double  DEFAULT_K_TAO      = -15.4930f;   // torque feedforward gain
static const double  DEFAULT_K_BEMF     = -1.5843f;    // back EMF compensation gain
static const double  DEFAULT_K_COULOMB  = -1.0;        // stiction compensation gain (in [0, 1])
static const double  DEFAULT_K_STIC     = 0.0;         // Coulomb friction compensation gain (in [0, 1])
static const double  DEFAULT_K_CP       = 0.6;         // positive Coulomb friction
static const double  DEFAULT_K_CN       = -0.6;        // negative Coulomb friction
static const double  DEFAULT_K_DITHER   = 12.0;        // dither signal amplitude (PWM)
static const double  DEFAULT_W_DITHER   = 0.9;         // dither signal frequency (rad/sec)

static const double  DEFAULT_INERTIA    = 0.001;
static const double  DEFAULT_DAMPING    = 0.01;
static const double  DEFAULT_STIFFNESS  = 0.02;

/**
LEFT ARM, BLACK ICUB
[1-step estimation] Joint 0 k_tau=-4.246522	    k_d=-2.844716
[1-step estimation] Joint 1 k_tau=-15.149277	k_d=-1.641094
[1-step estimation] Joint 2 k_tau=-18.076314	k_d=-1.550507
[1-step estimation] Joint 3 k_tau=-15.4930	    k_d=-1.5843

RIGHT LEG, WHITE ICUB
Joint 0     k_tau=-5.361510	k_dP=-4.380315	k_dN=-4.388992	k_cP=-8.317637	k_cN=-13.726499
**/

// coulomb friction right elbow red icub: +1, -2

// DEFAULT CONTROLLER SET POINTS
static const double DEFAULT_TAOD[]        = { 000.3 };

static const double GRAV_ACC = 9.81;

}

} // end namespace



#endif
