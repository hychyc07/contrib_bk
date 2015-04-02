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

#ifndef REACH_THREAD
#define REACH_THREAD

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

#include "iCub/safeReach/safeReachConstants.h"
#include "iCub/skinForceControl/skinManagerClient.h"
#include "iCub/skinForceControl/skinForceControlClient.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;
using namespace std;

namespace iCub
{

namespace safeReach
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
enum ReachPhase{ 
    reach_stop, reach_q0, reach_xd, reach_x0, track_target, REACH_PHASE_SIZE 
};
const string ReachPhase_s[REACH_PHASE_SIZE] = { 
    "reach_stop", "reach_q0", "reach_xd", "reach_x0", "track_target"
};

class safeReachThread: public yarp::os::RateThread
{
protected:
    // THREAD
    std::string             name;               // thread name
    VerbosityLevel          verbose;
    int                     period;             // thread period
    Semaphore               mutex;              // semaphore managing the access to all variables
    Status                  thread_status;      // the status of the thread (OK, ERROR)
                                                // as soon as the status is ERROR, the module calls stop() on this thread

    // SKIN and CONTROL interface
    skinManagerClient       *skinClient;
    skinForceControlClient  *controller;
    PolyDriver              *dd_gaze;
    IGazeControl            *igaze;
    int                     gazeCtrlContext;
        
    // Body part
    BodyPart            bodyPart;		// controlled body part (LEFT_ARM or RIGHT_ARM)
    
    // Controller variables
    int                 currentControlLaw;
    Vector              fd;
    Vector              xd;             // target end-effector position
    Vector              x0;             // starting end-effector position
    Vector              targetPos;      // target position when tracking target
    Vector              q0;             // starting joint configuration
    Vector              qRest;
    Vector              kp, kv, kf, kd;
    double              ks;
    double              trajTime;
    
    // Robot status
    SfcMonitorData      controllerStatus;
    Vector              f;              // force at the contact point
    Vector              x;              // position of ctrl point (root ref frame)
    Vector              q;
	
    // filters
    SignalStabilityAnalyzer *forceSignalAnalyzer;
    SignalStabilityAnalyzer *posSignalAnalyzer;
    SignalStabilityAnalyzer *jointSignalAnalyzer;
    ControllerImprovementAnalyzer *controlImprovement;
    bool isPosStable, isForceStable, isJointStable;
    
    // PORTS
    BufferedPort<Bottle>            *port_target_pos;   // input port reading target position
    BufferedPort<Vector>            *port_monitor;      // output port sending data for monitoring the thread
    BufferedPort<Bottle>            *port_info;         // output port sending info about the current status of the thread
    Vector monitorData;                                 // vector sent on the monitor output port

    ReachPhase currentReachPhase;
    bool reachPhaseChanged;

    void changeReachPhase(ReachPhase newRp) throw(){
        currentReachPhase = newRp;
        reachPhaseChanged = true;
    }

    void initReachPhase(ReachPhase rp) throw();
    void updateRobotStatus() throw();
    void updateCtrlRef(ReachPhase cp) throw();

    void sendMsg(const string &msg, const MsgType &type=MSG_INFO, bool toSend=false) throw();
    void sanityCheck() throw();
    void sendMonitorData() throw();

public:
    safeReachThread() throw();
    safeReachThread(std::string _name, int _period, BodyPart _bodyPart, VerbosityLevel _verbose=NO_VERBOSE) throw();
	  
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
    void startReach(){
        mutex.wait();
        changeReachPhase(reach_q0);
        mutex.post();
    }
    void stopReach(){
        mutex.wait();
        changeReachPhase(reach_stop);
        mutex.post();
    }
    void trackTarget(){
        mutex.wait();
        changeReachPhase(track_target);
        mutex.post();
    }
    Status setControlLaw(int cl){
        if(cl<0 || cl>2)
            return Status("Control Law out of bounds [0,2]");
        safeSet(currentControlLaw, cl);
        return Status();
    }
    

    // *******************************************************************************************************
    // *                                              GET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    T safeGet(const T& value){
        return iCub::skinForceControl::safeGet(value, mutex);
    }
    inline int getControlLaw(){ return safeGet(currentControlLaw); }
	inline Status getStatus(){ return safeGet(thread_status); }
    inline Vector getXd(){ return safeGet(xd); }
    inline Vector getX(){ return safeGet(x); }
    inline Vector getFd(){ return safeGet(fd); }
};


}

} // end namespace

#endif
