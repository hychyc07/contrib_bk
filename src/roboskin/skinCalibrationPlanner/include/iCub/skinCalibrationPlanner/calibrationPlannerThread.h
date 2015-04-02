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

#ifndef PLAN_THREAD
#define PLAN_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <list>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "iCub/skinCalibrationPlanner/calibrationPlannerConstants.h"
#include "iCub/skinForceControl/skinManagerClient.h"
#include "iCub/skinForceControl/skinForceControlClient.h"
#include "iCub/skinForceControl/skinCalibrationClient.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;
using namespace iCub::skinCalibration;
using namespace std;

namespace iCub
{

namespace skinCalibrationPlanner
{

/** 
* Calibration is divided into different phases:
* calib_stop: 
*   Do nothing, arm position control
* calib_startup: 
*   Starting phase of calibration, activate float ctrl, then wait until a contact on skin is detected, then move to calib_pick_dest
* calib_pick_dest:
*   Pick a new destination for the controller, then move to calib_moving
* calib_moving:
*   Wait until either the controller is stable (move to calib_pick_dest) or contact is lost (move to calib_contact_lost)
* calib_contact_lost:
*   Try to retrieve the contact with the environment 
* calib_restore:
*
* calib_end_eff:
*   Taxels are not calibrated so move end-effector back and forth
*/
enum CalibrationPhase{ 
    calib_stop, calib_startup, calib_pick_dest, calib_moving, calib_contact_lost, calib_restore, 
    calib_end_eff, calib_spiral, CALIBRATION_PHASE_SIZE 
};
const string CalibrationPhase_s[CALIBRATION_PHASE_SIZE] = { 
    "calib_stop", "calib_startup", "calib_pick_dest", "calib_moving", "calib_contact_lost","calib_restore", 
    "calib_end_eff", "calib_spiral"
};

class calibrationPlannerThread: public yarp::os::RateThread
{
protected:
    // THREAD
    std::string             name;               // planner name
    VerbosityLevel          verbose;
    int                     period;             // thread period
    Semaphore               mutex;              // semaphore managing the access to all controller variables
    Status                  thread_status;      // the status of the thread (OK, ERROR)
                                                // as soon as the status is ERROR, the module calls stop() on this thread

    // SKIN and CONTROL interface
    skinManagerClient       *skinClient;
    skinForceControlClient  *controller;
    skinCalibrationClient   *calibrator;
        
    // Skin Patch ID
    BodyPart            bodyPart;		// controlled body part (LEFT_ARM or RIGHT_ARM)
    SkinPart            skinPart;		// skin part that is being calibrated (FOREARM or UPPER_ARM)
    
    // Taxel positions
    vector<Vector>      taxelPos;           // 3d position of all the taxels of the interested skinPart
    Vector              taxelConf;          // taxel position estimation confidence
    double              totTaxelConf;       // sum of all the taxel confidences
	vector<unsigned int> taxelVisitCount;	// Count the number of visits for each taxel
	unsigned int		maxTaxelVisitCount;
    vector<list<unsigned int> > neighborsXtaxel;    // neighbors id for each taxel
    double              maxNeighDist;       // max distance between two neighbor taxels in m

    // Controller set points
    CtrlRef             ref;            // reference force and position

    // Robot status
    SfcMonitorData      controllerStatus;
    Vector              f;              // force at the contact point
    Vector              x;              // position of ctrl point (root ref frame)
    Vector              q;
	Vector				lastInContactRobotPose; // Joint angle values of the robot at the last stable contact
	Vector				firstInContactRobotPose;
	Vector				firstContactPosition;
	Vector				initialRobotPose;

    // status of calibration phase calib_end_eff
    Vector              initialXdEnv;
    Vector              secondXdEnv;
    enum { move_away, push_stronger, move_back} explorationPhase;
    struct{ 
        Vector pos;
        double confidence;
        vector<unsigned int> taxelList;
    } leastConfidenceXdEnv;

    // status of calibration phase calib_spiral
    double              currentMotionStep;              // each step is associated with a direction
    double              currentMotionSubstep;           // each step is divided into substeps of max length 0.01
    Vector              currentMotionDirection;         // motion direction is always orthogonal to contact force
    enum { spiral_forward, spiral_back} spiralPhase;

    SignalStabilityAnalyzer *forceSignalAnalyzer;
    SignalStabilityAnalyzer *posSignalAnalyzer;
    SignalStabilityAnalyzer *jointSignalAnalyzer;
    ControllerImprovementAnalyzer *controlImprovement;
	double				forceThreshold;
    bool                isForceStable;  // true if the measured force is stable
    bool                isPosStable;    // true if the measured position is stable
    bool                isJointStable;
    bool                isCalibrationOn;
    bool                firstAttempt;

    skinContact         contact;        // contact chosen for the controller
    bool                isSkinContact;  // true if at least one taxel is detecting touch
    bool                isInContact;    // true if either the skin or the F/T sensor are detecting a contact
	bool				isFirstTry;		// true if this is the first attempt to reach the desired Xd
	bool				isRestored;

	int					errorMinimizationTimeout;
	double				minError;
	double				overallMinError;

    double              feasibility;    // value assessing the feasibility of the current task (0 unfeasible, 1 perfectly feasible)
    
    
    // TIMESTAMPS
    double              taxelPosTimestamp;      // timestamp of the last update of the taxel positions
    double              isSkinContactTimestamp; // timestamp of the last time isSkinContact was set to true
    double              isInContactTimestamp;   // timestamp of the last time isInContact was set to true

    // PORTS
	Port							*port_ft_rpc;       // output port to connect to wholeBodyDynamics rpc port
    BufferedPort<Vector>            *port_monitor;      // output port sending data for monitoring the planner
    BufferedPort<Bottle>            *port_info;         // output port sending info about the current status of the planner
    BufferedPort<Vector>            *port_skin_dest;    // output port to connect to skinGui to see destination taxel

    Vector monitorData;                         // vector sent on the monitor output port
    Vector skinDestination;                     // vector sent on skin_dest output port

    CalibrationPhase currentCalibPhase;
    bool calibPhaseChanged;

    void changeCalibPhase(CalibrationPhase newCp) throw(){
        currentCalibPhase = newCp;
        calibPhaseChanged = true;
    }

    void initCalibPhase(CalibrationPhase cp) throw();
    void updateRobotStatus() throw();
    void updateCtrlRef(CalibrationPhase cp) throw();

    void sendMsg(const string &msg, const MsgType &type=MSG_INFO, bool toSend=false) throw();
    void sanityCheck() throw();
    void sendMonitorData() throw();

public:
    calibrationPlannerThread() throw();
    calibrationPlannerThread(std::string _name, int _period, SkinPart _skinPart, VerbosityLevel _verbose=NO_VERBOSE) throw();
	  
    bool threadInit();	
    void run();
    void threadRelease();

    // *******************************************************************************************************
    // *                                              SET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    void safeSet(T& var, const T& value){
        return iCub::skinForceControl::safeSet(var, value, mutex);
    }
    void startCalibration(){
        mutex.wait();
        changeCalibPhase(calib_startup);
        mutex.post();
    }
    void stopCalibration(){
        mutex.wait();
        changeCalibPhase(calib_stop);
        mutex.post();
    }
    

    // *******************************************************************************************************
    // *                                              GET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    T safeGet(const T& value){
        return iCub::skinForceControl::safeGet(value, mutex);
    }
	inline Status getStatus(){ return safeGet(thread_status); }
    inline Vector getXd(){ return safeGet(ref.x); }
    inline Vector getX(){ return safeGet(x); }
    inline Vector getFd(){ return safeGet(ref.f); }
    inline double getFeasibility() { return safeGet(feasibility); }
};


}

} // end namespace

#endif
