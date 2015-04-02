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
#include <fstream>
#include <stdexcept>
#include <vector>
#include <list>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "iCub/distanceImitation/distanceImitationConstants.h"
#include <iCub/distanceImitation/robot_interfaces.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{

namespace distanceImitation
{

/** 
* phase_calib: start float control
* phase_calib_check: start position control, move to q_home, do 1 trial to check calibration
* phase_pause: do nothing
* phase_running: run trials
* phase_running: run non-collaborative trials
*/
enum Phase{ phase_calib, phase_calib_check, phase_pause, phase_running, phase_running2, PHASE_SIZE };
const string Phase_s[PHASE_SIZE] = { "phase_calib", "phase_calib_check", "phase_pause", "phase_running", "phase_running2" };

class distanceImitationThread: public yarp::os::Thread
{
protected:
    // THREAD
    std::string             name;               // thread name
    std::string             robotName;          // robot name
    VerbosityLevel          verbose;
    Semaphore               mutex;              // semaphore managing the access to all variables
    bool                    isCalibrated;
    double                  startTime;
    Status                  thread_status;      // the status of the thread (OK, ERROR)
                                                // as soon as the status is ERROR, the module calls stop() on this thread

    // interfaces
    RobotInterfaces         *robot;
    PolyDriver              *dd_cart;
    PolyDriver              *dd_gaze;
    IGazeControl            *igaze;
    ICartesianControl       *icart;
    int                     gazeCtrlContext;
    int                     cartCtrlContext;
        
    // Body part
    BodyPart            bodyPart;		// controlled body part (LEFT_ARM or RIGHT_ARM)
    
    // Controller variables
    TrialInfo           trialInfo;      // information about the experiment
    Vector              xH, oH;         // 3d home hand pose taken in calibration phase
    Vector              x0, o0;         // 3d reference hand pose taken in calibration phase
    Vector              x1List;         // list of values of x1
    Vector              x2List;         // list of values of x2
    Vector              t1, t2;         // timestamps taken when x1/x2 are reached
    unsigned int        currentTrial;   // index of current trial
    unsigned int        totTrials;      // total number of trials
	
    ofstream            datafile;
    
    // PORTS
    BufferedPort<Vector>            *port_monitor;      // output port sending data for monitoring the thread
    BufferedPort<Bottle>            *port_info;         // output port sending info about the current status of the thread
    Vector monitorData;                                 // vector sent on the monitor output port

    Phase currentPhase;
    Phase previousPhase;
    bool phaseChanged;

    void changePhase(Phase newP) throw(){
        previousPhase = currentPhase;
        currentPhase = newP;
        phaseChanged = true;
    }

    void initPhase(Phase rp) throw();   // called when new phase starts
    void runPhase(Phase rp) throw();    // called every loop

    void sendMsg(const string &msg, const MsgType &type=MSG_INFO, bool toSend=false) throw();
    void sanityCheck() throw();

    void setArmInTorqueMode();
    void setArmInImpedanceMode();
    void moveArmToHome(bool gaze);
    void moveArmToHomeCart(bool gaze);
    void reachPose(Vector x, Vector o, bool gaze);
    void reachPose2(Vector x, Vector o, bool gaze);
    void computeTargetList();

public:
    distanceImitationThread() throw();
    distanceImitationThread(std::string _name, string _robotName, TrialInfo _trialInfo, VerbosityLevel _verbose=NO_VERBOSE) throw();
	  
    bool threadInit();	
    void run();
    void threadRelease();

    // *******************************************************************************************************
    // *                                              SET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    void safeSet(T& var, const T& value){
        return iCub::distanceImitation::safeSet(var, value, mutex);
    }
    Status startTrials()
    { 
        Status s;
        mutex.wait();
        if(currentPhase == phase_pause)
        {
            if(isCalibrated)
                changePhase(phase_running); 
            else
                s.addErrMsg("You cannot start the experiment because you have not calibrated yet.");
        }
        else
            s.addErrMsg("You cannot start the experiment unless you are in pause.");
        mutex.post(); 
        return s;
    }
    Status startTrials2()
    { 
        Status s;
        mutex.wait();
        if(currentPhase == phase_pause)
        {
            if(isCalibrated)
                changePhase(phase_running2); 
            else
                s.addErrMsg("You cannot start the experiment because you have not calibrated yet.");
        }
        else
            s.addErrMsg("You cannot start the experiment unless you are in pause.");
        mutex.post(); 
        return s;
    }
    Status pauseTrials()
    { 
        Status s;
        mutex.wait(); 
        if(currentPhase == phase_running || currentPhase==phase_running2)
            changePhase(phase_pause); 
        else
            s.addErrMsg("You cannot pause the experiment because it is not running.");
        mutex.post(); 
        return s;
    }
    Status calibStart()
    {  
        Status s;
        mutex.wait();
        if(currentPhase == phase_pause)
            changePhase(phase_calib); 
        else
            s.addErrMsg("You cannot start calibration unless you are in pause.");
        mutex.post(); 
        return s;
    }
    Status calibDone()
    {   
        Status s;
        mutex.wait(); 
        if(currentPhase == phase_calib)
            changePhase(phase_calib_check); 
        else
            s.addErrMsg("You cannot end the calibration because you have not started it yet.");
        mutex.post(); 
        return s;
    }
    

    // *******************************************************************************************************
    // *                                              GET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    T safeGet(const T& value){
        return iCub::distanceImitation::safeGet(value, mutex);
    }
	inline Status getStatus(){ return safeGet(thread_status); }
};


}

} // end namespace

#endif
