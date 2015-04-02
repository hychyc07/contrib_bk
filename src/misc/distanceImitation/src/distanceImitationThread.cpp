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

#include "iCub/distanceImitation/distanceImitationThread.h"
#include "iCub/distanceImitation/util.h"
#include "iCub/skinDynLib/common.h"
#include <iCub/ctrl/math.h>

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <yarp/math/Rand.h>
#include <iostream>
#include <set>
#include <algorithm>

using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::distanceImitation;
using namespace iCub::iKin;
using namespace iCub::ctrl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
distanceImitationThread::distanceImitationThread(string _name, string _robotName, TrialInfo _trialInfo, VerbosityLevel _verbose) throw()
: Thread(), name(_name), robotName(_robotName), trialInfo(_trialInfo), verbose(_verbose)
{
    //---------------------DEVICES--------------------------//
    bodyPart = RIGHT_ARM;
    robot = new RobotInterfaces(name.c_str(), robotName.c_str(), TORSO, RIGHT_ARM, LEFT_ARM);
    icart = 0;
    igaze = 0;

    //----------- INIT VARIABLES----------------//
    startTime = Time::now();
    currentPhase = phase_pause;
    isCalibrated = false;
    currentTrial = 0;
    iCubArm arm("right");
    arm.releaseLink(0);
    arm.releaseLink(1);
    arm.releaseLink(2);
    Vector poseHome = arm.EndEffPose(trialInfo.q_home.subVector(0,9)*CTRL_DEG2RAD);
    xH = poseHome.subVector(0,2);
    oH = poseHome.subVector(3,6);
    x0 = zeros(3);
    o0 = zeros(4);
    computeTargetList();
    if(trialInfo.x0.size()==7)
    {
        isCalibrated = true;
        x0 = trialInfo.x0.subVector(0,2);
        o0 = trialInfo.x0.subVector(3,6);
    }
    
    //---------------------PORTS-------------------------//	
    string slash = "/";
    port_monitor        = new BufferedPort<Vector>;
    port_info           = new BufferedPort<Bottle>;
    if(!port_monitor->open((slash+name+"/monitor:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the monitor:o port");
    if(!port_info->open((slash+name+"/info:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the info:o port");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool distanceImitationThread::threadInit()
{
    //---------------------DATA FILE-------------------------//
    string datatime = currentDateTime();
    string fn = trialInfo.filename+datatime+".txt";
    size_t found = fn.find(":");
    while(found!=string::npos)
    {
        fn.replace(found, 1,"_");
        found = fn.find(":");
    }
    datafile.open (fn.c_str());
    if(datafile.is_open() && datafile.good())
    {
        sendMsg("Opening data file "+fn);
        datafile<< "Datatime\t"<< datatime<< endl;
        if(!writeTrialInfoToFile(datafile, trialInfo))
        {
            sendMsg("Problem writing trial info to file");
            return false;
        }
        datafile<< "For each trial the following data are reported:\nid t1 x1d x1 t2 x2d x2\n";
        datafile<< endl;
    }
    else
    {
        sendMsg("Problems opening data file "+fn);
        return false;
    }

    string slash = "/";
    //---------------------ROBOT INTERFACES-------------------------//
    sendMsg("Going to initialize robot interfaces...");
    if(robot->init())
    {
        sendMsg("Initialization succeeded!");
    }
    else
    {
        sendMsg("Unable to communicate with robot. Stopping the module.", MSG_ERROR);
        return false;
    }

    //---------------------CARTESIAN CONTROLLER--------------------------//
    sendMsg("Going to initialize cartesian controller client...");
    Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote", (slash+robotName+"/cartesianController/right_arm").c_str());
    option.put("local", ("/"+name+"/cart_right_arm").c_str());
 
    dd_cart = new PolyDriver(option);
    icart=NULL;
    if (dd_cart->isValid() && dd_cart->view(icart)) 
    {
        sendMsg("Initialization succeeded!");
        icart->storeContext(&cartCtrlContext);
        icart->stopControl();
        // set torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;
        newDof.setSubvector(0, cat(1,0,1)); // only torso yaw enabled
        icart->setDOF(newDof,curDof);
        // set tracking mode
        icart->setTrackingMode(false);
        icart->setInTargetTol(CART_CTRL_TOL);

        Vector curRestPos, curRestWeights;
        icart->getRestPos(curRestPos);
        icart->getRestWeights(curRestWeights);
        //sendMsg("Joint rest positions: "+string(curRestPos.toString(1).c_str()));  // [0 0 0 0 0 0 0 0 0 0] will be printed out
        //sendMsg("Joint rest positions: "+string(curRestWeights.toString(1).c_str()));  // [1 1 1 0 0 0 0 0 0 0] will be printed out
    }
    else
    {
        sendMsg("Unable to communicate with cartesian controller. Stopping the module.", MSG_ERROR);
        return false;
    }

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
        igaze->setTrackingMode(true);
        igaze->blockNeckRoll(0.0);
        igaze->setNeckTrajTime(1.0);
        igaze->setEyesTrajTime(0.8);
        sendMsg("Initialization succeeded!");
    }
    else
		sendMsg("WARNING: unable to create gaze device driver! Going on anyway.");

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::threadRelease()
{
    sendMsg("distanceImitation Thread release.\n");
    datafile.close();
    if(igaze){  igaze->stopControl(); igaze->restoreContext(gazeCtrlContext); delete igaze; igaze=0; }
    if(icart){  icart->stopControl(); icart->restoreContext(cartCtrlContext); delete icart; icart=0; }
	if(port_monitor)        { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
    if(port_info)           { port_info->interrupt(); port_info->close(); delete port_info;  port_info = 0;}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::run()
{
    Phase p;
    bool pChange;
    while(thread_status && !isStopping())
    {
        // check everything is working fine
        sanityCheck();
    
        mutex.wait();
        p = currentPhase;
        pChange = phaseChanged;
        if(pChange)
            phaseChanged = false;
        mutex.post();

        if(pChange){
            // if the ctrl mode has changed, initialize the new ctrl mode
            sendMsg("Phase just changed to "+Phase_s[p], MSG_INFO);
            //endPhase(previousPhase);
            initPhase(p);
        }
    
        runPhase(p);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::initPhase(Phase p) throw()
{
    switch(p)
    {
    case phase_calib:
        moveArmToHome(true);
        setArmInTorqueMode();
        break;

    case phase_calib_check:
        if(icart->getPose(x0, o0))
        {
            sendMsg("Calibration done: "+string(x0.toString(2).c_str()));
            isCalibrated = true;
            datafile<<"x0\t"<< x0.toString(3).c_str()<< o0.toString(3).c_str()<< endl;
        }
        else
            sendMsg("Error, calibration failed.");
        setArmInImpedanceMode();
        break;

    case phase_pause:
        icart->stopControl();
        break;

    case phase_running:
        igaze->setTrackingMode(true);
        moveArmToHome(true);
        break;

    case phase_running2:
        moveArmToHome(true);
        Time::delay(1.5);
        igaze->setTrackingMode(false);
        igaze->stopControl();
        break;

    default:
        sendMsg("Unknown phase in initPhase: "+toString(p), MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::runPhase(Phase p) throw()
{
    switch(p)
    {
    case phase_calib:
        break;

    case phase_calib_check:
        Time::delay(1.0);
        moveArmToHomeCart(true);
        datafile << "0 ";
        reachPose(x0, o0, true);
        reachPose2(x0-cat(0,0.1,0), o0, true);
        moveArmToHomeCart(true);
        datafile<< endl;
        mutex.wait();
        changePhase(phase_pause);
        mutex.post();
        break;

    case phase_pause:
        Time::delay(2.0);
        break;

    case phase_running:
    case phase_running2:
        currentTrial++;
        if(currentTrial >= totTrials)
        {
            datafile.close();
            sendMsg("Experiment finished.");
            mutex.wait();
            changePhase(phase_pause);
            mutex.post();
            return;
        }
        //sendMsg("Trial "+toString(currentTrial)+" out of "+toString(totTrials)+" (distance "+toString(x2List[currentTrial]-x1List[currentTrial])+" m)");
        // in non-collaborative experiment wait for the user to press 'return'
        if(p==phase_running2){
            sendMsg("Press return to start the trial "+toString(currentTrial));
            cin.get();
        }
        datafile << currentTrial<< " ";
        reachPose(x0-cat(0,x1List[currentTrial],0), o0, p==phase_running);
        reachPose2(x0-cat(0,x2List[currentTrial],0), o0, p==phase_running);
        moveArmToHomeCart(p==phase_running);
        datafile << endl;
        datafile.flush();
        Time::delay(trialInfo.T);
        break;

    default:
        sendMsg("Unknown phase in runPhase: "+toString(p), MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::reachPose(Vector xd, Vector od, bool gaze)
{
    Vector x, o;
    while(!icart->getPose(x, o))
    {
        sendMsg("Cannot get current pose. Gonna wait a while and try again.", MSG_WARNING);
        Time::delay(0.1);
    }
    double tt = min(MAX_TRAJ_TIME, max(MIN_TRAJ_TIME, norm(x-xd) / trialInfo.speed));
    //sendMsg("Moving to "+string(xd.toString(2).c_str())+" in time "+toString(tt));

    if(gaze && igaze){
        igaze->lookAtFixationPoint(xd);
        Time::delay(0.5);
    }
    
    icart->setInTargetTol(5.0*CART_CTRL_TOL);
    icart->goToPoseSync(xd+cat(0,0,0.03), od, tt);
    icart->waitMotionDone(0.1, 5.0*tt);

    icart->setInTargetTol(CART_CTRL_TOL);
    icart->goToPoseSync(xd, od, MIN_TRAJ_TIME);
    icart->waitMotionDone(0.1, 5.0*MIN_TRAJ_TIME);

    datafile << Time::now()-startTime << " ";
    datafile << xd.toString(4) << " ";
    while(!icart->getPose(x, o))
        Time::delay(0.1);
    datafile << x.toString(4) << " ";

    //icart->setInTargetTol(5.0*CART_CTRL_TOL);
    //icart->goToPoseSync(xd+cat(0,0,0.03), od, tt);
    //icart->waitMotionDone(0.1, 5.0*tt);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::reachPose2(Vector xd, Vector od, bool gaze)
{
    Vector x, o;
    while(!icart->getPose(x, o))
    {
        sendMsg("Cannot get current pose. Gonna wait a while and try again.", MSG_WARNING);
        Time::delay(0.1);
    }
    if(gaze && igaze)
        igaze->lookAtFixationPoint(xd);
    
    double tt = MIN_TRAJ_TIME;
    Vector x3 = x+cat(0,0,0.75*norm(x-xd));
    Vector p = xd-x3+cat(0,0,0.02);
    int stepNum = 10;
    icart->setInTargetTol(5.0*CART_CTRL_TOL);
    for(double alpha = 0.0; alpha<=1.0; alpha+=1.0/stepNum)
    {
        icart->goToPoseSync(x3+alpha*p, od, tt);
        Time::delay(1.5*tt/stepNum);
    }
    icart->waitMotionDone(0.1, 5.0*tt);
    icart->setInTargetTol(CART_CTRL_TOL);
    icart->goToPoseSync(xd, od, tt);
    icart->waitMotionDone(0.1, 5.0*tt);

    datafile << Time::now()-startTime << " ";
    datafile << xd.toString(4) << " ";
    while(!icart->getPose(x, o))
        Time::delay(0.1);
    datafile << x.toString(4) << " ";

    //icart->setInTargetTol(5.0*CART_CTRL_TOL);
    //icart->goToPoseSync(xd+cat(0,0,0.03), od, MIN_TRAJ_TIME);
    //icart->waitMotionDone(0.1, 2.0*MIN_TRAJ_TIME);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::moveArmToHomeCart(bool gaze)
{
    Vector x, o;
    while(!icart->getPose(x, o))
    {
        sendMsg("Cannot get current pose. Gonna wait a while and try again.", MSG_WARNING);
        Time::delay(0.1);
    }
    double tt = min(MAX_TRAJ_TIME, max(MIN_TRAJ_TIME, norm(x-xH) / trialInfo.speed));
    //sendMsg("Moving to "+string(xH.toString(2).c_str())+" in time "+toString(tt));

    if(gaze && igaze)
        igaze->lookAtFixationPoint(trialInfo.gaze_home);
    
    icart->setInTargetTol(10.0*CART_CTRL_TOL);
    icart->goToPoseSync(xH, oH, tt);
    icart->waitMotionDone(0.1, 5.0*tt);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::moveArmToHome(bool gaze)
{
    //sendMsg("Moving arm to home");//+string(trialInfo.q_home.toString(1).c_str()));
    icart->stopControl();
    if(gaze && igaze)
        igaze->lookAtFixationPoint(trialInfo.gaze_home);
    for(int j=0; j<3; j++)
        robot->ipos[TORSO]->positionMove(j, trialInfo.q_home[2-j]);
    
    robot->ipos[bodyPart]->setRefSpeed(15, 100);
    for(int j=3; j<N_JOINT; j++)
        robot->ipos[bodyPart]->positionMove(j-3, trialInfo.q_home[j]);
    
    // move left arm
    robot->ipos[LEFT_ARM]->positionMove(0, 10);
    robot->ipos[LEFT_ARM]->positionMove(1, 25);
    robot->ipos[LEFT_ARM]->positionMove(2, 0);
    robot->ipos[LEFT_ARM]->positionMove(3, 15);

    bool motionDone = false;
    while(!motionDone)
    {
        robot->ipos[bodyPart]->checkMotionDone(&motionDone);
        Time::delay(0.1);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::setArmInTorqueMode()
{
    for(int i=0; i<5; i++)
        robot->icmd[bodyPart]->setTorqueMode(i);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::setArmInImpedanceMode()
{
    for(int i=0; i<5; i++)
        robot->icmd[bodyPart]->setPositionMode(i);
        //robot->icmd[bodyPart]->setImpedancePositionMode(i);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::sanityCheck() throw()
{

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::sendMsg(const string &msg, const MsgType &type, bool toSend) throw()
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
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void distanceImitationThread::computeTargetList()
{
    totTrials = trialInfo.N * (trialInfo.k_max+1);
    x1List.resize(totTrials);
    x2List.resize(totTrials);
    vector<int> kappas(totTrials);

    for (unsigned int k=0; k<trialInfo.k_max; k++) 
        for( unsigned int i=0; i<trialInfo.N; i++)
            kappas[k*trialInfo.N + i] = k;        
    random_shuffle( kappas.begin(), kappas.end());
    
    for( unsigned int i=0; i<totTrials; i++)
    {
        x1List[i] = Rand::scalar(0.5e-2, 3.5e-2);
        x2List[i] = x1List[i] + trialInfo.d_min + kappas[i]*trialInfo.step;
    }
}
