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

#include "iCub/safeReach/safeReachThread.h"
#include "iCub/safeReach/util.h"
#include <iCub/skinForceControl/utilRobotics.h>
#include "iCub/skinDynLib/common.h"
#include <iCub/ctrl/math.h>

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <iostream>
#include <set>
#include <algorithm>

using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::safeReach;
using namespace iCub::iKin;
using namespace iCub::ctrl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
safeReachThread::safeReachThread(string _name, int _period, BodyPart _bodyPart, VerbosityLevel _verbose) throw()
: RateThread(_period), name(_name), period(_period), bodyPart(_bodyPart), verbose(_verbose)
{
    //---------------------DEVICES--------------------------//
    //skinClient      = new skinManagerClient(name.c_str());
    controller      = new skinForceControlClient(name.c_str());

    //----------- INIT VARIABLES----------------//
    reachPhaseChanged   = true;
    currentReachPhase   = reach_stop;
    currentControlLaw   = 0;

    kp = DEFAULT_KP;
    kv = DEFAULT_KV;
    kf = DEFAULT_KF;
    kd = DEFAULT_KD;
    ks = DEFAULT_KS;
    fd  = DEFAULT_FD;
    targetPos = x0 = xd  = (bodyPart==LEFT_ARM ? DEFAULT_XD_LEFT : DEFAULT_XD_RIGHT);
    q0 = DEFAULT_Q0;
    qRest = DEFAULT_Q_REST;

    forceSignalAnalyzer = new SignalStabilityAnalyzer(STABILITY_TIME_WINDOW_F, period*1e-3, F_STAB_THR,3);
    posSignalAnalyzer   = new SignalStabilityAnalyzer(STABILITY_TIME_WINDOW_X, period*1e-3, X_STAB_THR,3);
    jointSignalAnalyzer = new SignalStabilityAnalyzer(STABILITY_TIME_WINDOW_Q, period*1e-3, Q_STAB_THR,N_JOINT);
    controlImprovement  = new ControllerImprovementAnalyzer(STABILITY_TIME_WINDOW_X, period*1e-3, MIN_POS_ERR_IMPROVEMENT);
    
    //---------------------PORTS-------------------------//	
    string slash = "/";
    port_target_pos     = new BufferedPort<Bottle>;
    port_monitor        = new BufferedPort<Vector>;
    port_info           = new BufferedPort<Bottle>;
    if(!port_target_pos->open((slash+name+"/target:i").c_str()))
        thread_status.addErrMsg("It was not possible to open the target:i port");
	if(!port_monitor->open((slash+name+"/monitor:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the monitor:o port");
    if(!port_info->open((slash+name+"/info:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the info:o port");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool safeReachThread::threadInit()
{
    /*sendMsg("Going to initialize skinManagerClient...");
    while(!skinClient->init())
    {
        sendMsg("Error while initializing the skinManagerClient. Going to wait a while and try again.", MSG_WARNING);
        Time::delay(1.0);
        if(!thread_status) return false;
    }
    sendMsg("SkinManagerClient initialization succeeded!");*/

    //---------------------GAZE CONTROLLER--------------------------//
    sendMsg("Going to initialize gaze controller...");
	Property OptionGaze;
    OptionGaze.put("device", "gazecontrollerclient");
    OptionGaze.put("remote", "/iKinGazeCtrl");
    OptionGaze.put("local", ("/"+name+"/gaze_client").c_str());         
    dd_gaze = new PolyDriver(OptionGaze);
    if (dd_gaze->isValid() && dd_gaze->view(igaze)) 
    {
        igaze->storeContext(&gazeCtrlContext);
        sendMsg("Initialization succeeded!");
    }
    else
		sendMsg("WARNING: unable to create gaze device driver! Going on anyway.");

    sendMsg("Going to initialize skinForceControlClient...");
    while(!controller->init())
    {
        sendMsg("Error while initializing the skinForceControlClient. Going to wait a while and try again.", MSG_WARNING);
        Time::delay(1.0);
        if(!thread_status) return false;
    }

    sendMsg("Initialization succeeded! Going to read contact...");
    while(!controller->getStreamData(controllerStatus))
    {
        sendMsg("Error while reading streaming data with skinForceControlClient. Going to wait a while and try again.", MSG_WARNING);
        Time::delay(1.0);
        if(!thread_status) return false;
    }
    sendMsg("Contacts read!");

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::threadRelease()
{
    sendMsg("safeReach Thread release.\n");
    int i=0;
    if(forceSignalAnalyzer){    delete forceSignalAnalyzer; forceSignalAnalyzer=0; }
    printf("%d", i++);
    if(posSignalAnalyzer){      delete posSignalAnalyzer; posSignalAnalyzer=0; }
    printf("%d", i++);
    if(controlImprovement){     delete controlImprovement; controlImprovement=0; }
    printf("%d", i++);
    //if(skinClient){             delete skinClient; skinClient=0; }
    //printf("%d", i++);
    if(controller){             delete controller; controller=0; }
    printf("%d", i++);

	if(port_monitor)        { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
	printf("%d", i++);
    if(port_info)           { port_info->interrupt(); port_info->close(); delete port_info;  port_info = 0;}
    printf("%d", i++);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::run()
{
    // check everything is working fine
    sanityCheck();
    
    mutex.wait();
    {
        if( !thread_status )
            return mutex.post();
        
        if(reachPhaseChanged)
        {
            sendMsg("Phase changed to: "+ReachPhase_s[currentReachPhase], MSG_INFO, true);
            initReachPhase(currentReachPhase);
            reachPhaseChanged = false;
        }

        // CONTROL LOOP
        updateRobotStatus();
        updateCtrlRef(currentReachPhase);

        sendMonitorData();
    }
    mutex.post();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::initReachPhase(ReachPhase rp) throw()
{
	switch(rp)
    {
        case reach_stop:
            controller->stop(); 
            break;
        case reach_q0:
			{
                jointSignalAnalyzer->reset();
                controller->setQd(q0);
				controller->startControl(JPOS_CTRL);
                break;
			}
        case reach_xd:
			{
                posSignalAnalyzer->reset();
                forceSignalAnalyzer->reset();
                controlImprovement->reset();
                controller->setXd(xd);
                controller->setCtrlPnt(N_JOINT-1, zeros(3));
                controller->setQd(qRest);
				controller->startControl(ctrlLaws[currentControlLaw]);
                break;
			}
        case reach_x0:
			{
                posSignalAnalyzer->reset();
                forceSignalAnalyzer->reset();
                controlImprovement->reset();
                controller->setXd(x0);
                controller->setCtrlPnt(N_JOINT-1, zeros(3));
                controller->setQd(qRest);
				controller->startControl(ctrlLaws[currentControlLaw]);
                break;
			}
        case track_target:
            {
                controller->setXd(targetPos);
                controller->setCtrlPnt(N_JOINT-1, zeros(3));
                controller->setQd(qRest);
                controller->startControl(ctrlLaws[currentControlLaw]);
                break;
            }
        default:    
            sendMsg("Unknown reach phase in initReachPhase: "+toString(rp), MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::updateRobotStatus() throw()
{
    // *** READ POSITION OF CTRL POINT AND CONTACT FORCE
    if(!controller->getStreamData(controllerStatus)) sendMsg("Error while reading controller status.", MSG_WARNING);
    x = controllerStatus.getX();
    f = controllerStatus.getForce();
    q = controllerStatus.getJointAng();
    
    // CHECK IF THEY ARE STABLE
    isPosStable = posSignalAnalyzer->addSample(x);
    isForceStable = forceSignalAnalyzer->addSample(f);
    isJointStable = jointSignalAnalyzer->addSample(q);
    controlImprovement->addSample(xd-x);
    
    static int counter=0;
    if(counter++ % 1000 == 0)
    {
        printf(isPosStable?"Pos STABLE\t":"Pos NOT STABLE\t");
        printf(isForceStable?"Force STABLE\t":"Force NOT STABLE\t");
        printf(isJointStable?"Joint STABLE\t":"Joint NOT STABLE\t");
        printf(controlImprovement->isImproving()?"Ctrl IMPROVING\n":"Ctrl NOT IMPROVING\n");
        if(controlImprovement->isImproving())
            printf("Time left: %.3f\n", controlImprovement->getRemainingTime());
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::updateCtrlRef(ReachPhase rp) throw()
{
	switch(rp)
    {
    case reach_stop:
        break;
    case reach_q0:
        {
            if(isJointStable)
            {
                x0 = controllerStatus.getX();
                changeReachPhase(reach_xd);
            }
            break;
		}
    case reach_xd:
        {
            if(isPosStable && isForceStable && !controlImprovement->isImproving())
                changeReachPhase(reach_x0);
            break;
        }
    case reach_x0:
        {
            if(isPosStable && isForceStable && !controlImprovement->isImproving())
                changeReachPhase(reach_xd);
            break;
        }
    case track_target:
        {
            if (Bottle *targetPosNew = port_target_pos->read(false))
            {
                if (targetPosNew->size()>6 && targetPosNew->get(6).asDouble()==1.0)
                {
                    Vector fp = cat(targetPosNew->get(0).asDouble(), targetPosNew->get(1).asDouble(), targetPosNew->get(2).asDouble(), 1.0);
                    if (!gsl_isnan(fp[0]) && !gsl_isnan(fp[1]) && !gsl_isnan(fp[2]))
                    {
                        Vector x,o;
                        igaze->getLeftEyePose(x,o);
                        //gazeCtrl->getRightEyePose(x,o);
                        Matrix T=axis2dcm(o);
                        T.setSubcol(x, 0, 3);
                        rototranslate(T,fp,targetPos);
                        //printf("Target position: %s\n", targetPos.toString(2).c_str());
                        controller->setXd(targetPos);
                    }
                    else
                        sendMsg("Target position is not a number.", MSG_WARNING);
                }
            }
            break;
        }
    default:    
        sendMsg("Unknown phase in updateCtrlRef", MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::sanityCheck() throw()
{

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::sendMonitorData() throw()
{
    // if nobody is reading do not write
    if(port_monitor->getOutputCount()==0)
        return;
    int index = 0;
    monitorData.resize(0);
    monitorData.zero();
    port_monitor->prepare() = monitorData;
    port_monitor->write();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void safeReachThread::sendMsg(const string &msg, const MsgType &type, bool toSend) throw()
{
    if(type==MSG_ERROR)
        thread_status.addErrMsg(msg);
    printf("%s: %s\n", MsgType_s[type].c_str(), msg.c_str());
    if(toSend)
    {
        Bottle& b = port_info->prepare();
        b.clear();
        b.addString(msg.c_str());
        port_info->write();
    }
}
