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

#include "iCub/skinCalibrationPlanner/calibrationPlannerThread.h"
#include "iCub/skinCalibrationPlanner/util.h"
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
using namespace iCub::skinCalibrationPlanner;
using namespace iCub::iKin;
using namespace iCub::ctrl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
calibrationPlannerThread::calibrationPlannerThread(string _name, int _period, SkinPart _skinPart, VerbosityLevel _verbose) throw()
: RateThread(_period), name(_name), period(_period), skinPart(_skinPart), verbose(_verbose)
{
    //---------------------DEVICES--------------------------//
    skinClient      = new skinManagerClient(name.c_str());
    controller      = new skinForceControlClient(name.c_str());
    calibrator      = new skinCalibrationClient(name.c_str());

    //----------- INIT VARIABLES----------------//
    maxNeighDist        = DEFAULT_MAX_NEIGH_DIST;
    bodyPart            = skinDynLib::getBodyPart(skinPart);

    calibPhaseChanged   = true;
    currentCalibPhase   = calib_stop;

    feasibility         = 1.0;
    contact             = skinContact(bodyPart, skinPart, 0, zeros(3), zeros(3), 0, 0.0);
    isSkinContact       = false;
    isInContact         = false;
    isCalibrationOn     = true;
    
	forceThreshold      = FORCE_THRESHOLD;
    forceSignalAnalyzer = new SignalStabilityAnalyzer(STABILITY_TIME_WINDOW, period*1e-3, F_STAB_THR,3);
    posSignalAnalyzer   = new SignalStabilityAnalyzer(STABILITY_TIME_WINDOW, period*1e-3, X_STAB_THR,3);
    jointSignalAnalyzer = new SignalStabilityAnalyzer(CALIB_STABILITY_TIME_WINDOW, period*1e-3, Q_STAB_THR,3);
    controlImprovement  = new ControllerImprovementAnalyzer(STABILITY_TIME_WINDOW, period*1e-3, MIN_POS_ERR_IMPROVEMENT);
    
	errorMinimizationTimeout = int(STABILITY_TIME_WINDOW / (period*1e-3));

    ref.f = ref.x       = zeros(3);
    ref.taxelId         = -1;
    leastConfidenceXdEnv.pos = zeros(3);
    leastConfidenceXdEnv.confidence = DBL_MAX;

    //---------------------PORTS-------------------------//	
    string slash = "/";
    port_monitor        = new BufferedPort<Vector>;
    port_info           = new BufferedPort<Bottle>;
	port_ft_rpc			= new Port();
    port_skin_dest      = new BufferedPort<Vector>;
	if(!port_monitor->open((slash+name+"/monitor:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the monitor:o port");
    if(!port_info->open((slash+name+"/info:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the info:o port");
    if(!port_skin_dest->open((slash+name+"/skin_dest:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the skin_dest:o port");
	if(!port_ft_rpc->open((slash+name+"/FTrpc:o").c_str()))
		thread_status.addErrMsg("It was not possible to open FTrpc:o port");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool calibrationPlannerThread::threadInit()
{
#ifdef _SIMULATION
    taxelPos.resize(192, zeros(3));
#else
    while(!skinClient->init())
    {
        sendMsg("Error while initializing the skinManagerClient. Going to wait a while and try again.", MSG_WARNING);
        Time::delay(1.0);
        if(!thread_status) return false;
    }
    sendMsg("SkinManagerClient initialization succeeded!");
    // get taxel positions and compute neighbors for each taxel
    taxelPos = skinClient->getTaxelPositions(skinPart);
    taxelConf = skinClient->getPoseConfidences(skinPart);
    totTaxelConf = norm1(taxelConf);
    sendMsg("Poses are now up to date");
	taxelVisitCount.resize(taxelPos.size());
	maxTaxelVisitCount = 1;
    skinDestination.resize(taxelPos.size());
#endif
    computeNeighbors(taxelPos, maxNeighDist, neighborsXtaxel);
    sendMsg("Neighbors computed");
    taxelPosTimestamp = Time::now();

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
    contact = controllerStatus.getContact();
        
    sendMsg("Going to initialize SkinCalibrationClient...");
    if(!calibrator->init())
    {
        sendMsg("Error while initializing the skinCalibrationClient.", MSG_WARNING);
        Time::delay(1.0);
    }
    else
        sendMsg("Initialization succeeded!");
    
#ifndef _SIMULATION
    sendMsg("Going to connect to wholeBodyDynamics rpc port...");
	if(!Network::connect(port_ft_rpc->getName().c_str(),"/wholeBodyDynamics/rpc:i"))
    {
        sendMsg("Connection to wholeBodyDynamics rpc failed. Stopping the thread", MSG_ERROR);
        return false;
    }
    sendMsg("Connection succeeded!");
#endif

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::threadRelease()
{
    sendMsg("calibrationPlanner Thread release.\n");
    Status s;

    if(forceSignalAnalyzer){    delete forceSignalAnalyzer; forceSignalAnalyzer=0; }
    if(posSignalAnalyzer){      delete posSignalAnalyzer; posSignalAnalyzer=0; }
    if(controlImprovement){     delete controlImprovement; controlImprovement=0; }
    if(skinClient){     delete skinClient; skinClient=0; }
    if(controller){     delete controller; controller=0; }
    if(calibrator){     delete calibrator; calibrator=0; }

	if(port_ft_rpc)			{ port_ft_rpc->interrupt(); port_ft_rpc->close(); delete port_ft_rpc;  port_ft_rpc = 0;}
    if(port_monitor)        { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
    if(port_info)           { port_info->interrupt(); port_info->close(); delete port_info;  port_info = 0;}
    if(port_skin_dest)      { port_skin_dest->interrupt(); port_skin_dest->close(); delete port_skin_dest;  port_skin_dest = 0;}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::run()
{
    // check everything is working fine
    sanityCheck();
    
    mutex.wait();
    {
        if( !thread_status )
            return mutex.post();
        
        if(calibPhaseChanged)
        {
            sendMsg("Calibration phase changed to: "+CalibrationPhase_s[currentCalibPhase]);
            initCalibPhase(currentCalibPhase);
            calibPhaseChanged = false;
        }

        // CONTROL LOOP
        updateRobotStatus();
        updateCtrlRef(currentCalibPhase);

        sendMonitorData();
    }
    mutex.post();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::initCalibPhase(CalibrationPhase cp) throw()
{
	switch(cp)
    {
        case calib_stop:            controller->stop(); break;
        case calib_startup:
			{
				if(!controller->getStreamData(controllerStatus)) sendMsg("Error while reading controller status", MSG_WARNING);
				initialRobotPose = controllerStatus.getJointAng();
				controller->startControl(FLOAT_CTRL); break;
			}
        case calib_end_eff:
            {
                controlImprovement->reset();
                initialXdEnv = ref.xEnv;
                secondXdEnv = initialXdEnv + rotateOntoPlane(MOTION_STEP, controllerStatus.getForce());
                explorationPhase = move_back;
                controller->startControl(CONTACT_CTRL);
                break;
            }
        case calib_spiral:
            {
                controlImprovement->setTimeWindow(SPIRAL_STABILITY_TIME_WINDOW);
                controlImprovement->reset();
                initialXdEnv = ref.xEnv;
                currentMotionDirection = versor(rotateOntoPlane(Rand::vector(3), controllerStatus.getForce()));
                currentMotionStep = SPIRAL_INITIAL_MOTION_STEP;
                currentMotionSubstep = 0.0;
                spiralPhase = spiral_forward;
                controller->startControl(CONTACT_CTRL);
                break;
            }
        case calib_pick_dest:       break;
        case calib_moving:
			{  
				errorMinimizationTimeout = int(STABILITY_TIME_WINDOW / (period*1e-3));
				minError = DBL_MAX;
				overallMinError = DBL_MAX;
				controller->startControl(CONTACT_CTRL); break;
			}
        case calib_contact_lost:
			{
				sendMsg("Contact lost!");
				isSkinContact = false;
				isInContact = false;
                controller->setQd(lastInContactRobotPose);
				posSignalAnalyzer->reset();
				forceSignalAnalyzer->reset();
				forceThreshold = FORCE_CONTACT_THRESHOLD;
				controller->startControl(JPOS_CTRL);
				break;
			}
		case calib_restore:
			{
				isSkinContact = false;
				isInContact = false;
				isRestored = false;
				ref.taxelId = -1;
                controller->setQd(initialRobotPose);
				posSignalAnalyzer->reset();
				forceSignalAnalyzer->reset();
				controller->startControl(JPOS_CTRL);
				break;
			}
        default:    sendMsg("Unknown calibration phase in iniCalibPhase: "+toString(cp), MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::updateRobotStatus() throw()
{
    // *** READ POSITION OF CTRL POINT AND CONTACT FORCE
    if(!controller->getStreamData(controllerStatus)) sendMsg("Error while reading controller status.", MSG_WARNING);
    x = controllerStatus.getX();
    f = controllerStatus.getForce();
    q = controllerStatus.getJointAng();
    // CHECK IF THEY ARE STABLE
    isPosStable = posSignalAnalyzer->addSample(x);
    isForceStable = forceSignalAnalyzer->addSample(f);
    isJointStable = jointSignalAnalyzer->addSample(cat(q[3],q[4],q[6]));

    if(isCalibrationOn && !isJointStable)
    {
        calibrator->pauseCalibration();
        isCalibrationOn = false;
    }
    else if(!isCalibrationOn && isJointStable)
    {
        calibrator->startCalibration();
        isCalibrationOn = true;
    }

    // *** READ CONTACT ***
    contact = controllerStatus.getContact();
    // if at least one taxel is active then the arm is in contact
    isSkinContact = contact.getActiveTaxels()>0 && contact.getSkinPart()==skinPart;
    if(isSkinContact)
    {
        isSkinContactTimestamp = Time::now();
        // check average confidence of active taxels
        double avgConf = 0.0;
        for(unsigned int i=0; i<contact.getActiveTaxels(); i++)
            avgConf += taxelConf[contact.getTaxelList()[i]];
        avgConf /= contact.getActiveTaxels();
        if(avgConf<leastConfidenceXdEnv.confidence)
        {
            leastConfidenceXdEnv.pos = controllerStatus.getX();
            leastConfidenceXdEnv.confidence = avgConf;
            leastConfidenceXdEnv.taxelList = contact.getTaxelList();
        }
    }
        
    // if no taxel is active but the force norm is above threshold the arm is in contact
    isInContact = isSkinContact || contact.getForceModule()>forceThreshold;
    if(isInContact)
        isInContactTimestamp = Time::now();

    // *** UPDATE TAXEL POSITIONS
    if(Time::now()-taxelPosTimestamp >= TAXEL_POS_UPDATE_PERIOD)
    {
        vector<Vector> posTemp = skinClient->getTaxelPositions(skinPart);
        if(posTemp.size()>0)
        {
            taxelPos = posTemp;
            computeNeighbors(taxelPos, maxNeighDist, neighborsXtaxel);
        }
        else
            sendMsg("Error while trying to read taxel positions.", MSG_WARNING);
            
        Vector confTemp = skinClient->getPoseConfidences(skinPart);
        if(confTemp.size()>0)
        {
            taxelConf = confTemp;
            totTaxelConf = norm1(taxelConf);
            // update average confidence of active taxels
            if(leastConfidenceXdEnv.taxelList.size()>0)
            {
                double oldConf = leastConfidenceXdEnv.confidence;
                double avgConf = 0.0;
                for(unsigned int i=0; i<leastConfidenceXdEnv.taxelList.size(); i++)
                    avgConf += taxelConf[leastConfidenceXdEnv.taxelList[i]];
                leastConfidenceXdEnv.confidence = avgConf / leastConfidenceXdEnv.taxelList.size();
            }
        }
        else
            sendMsg("Error while trying to read pose confidences.", MSG_WARNING);
        
        taxelPosTimestamp = Time::now();
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::updateCtrlRef(CalibrationPhase cp) throw()
{
	switch(cp)
    {
    case calib_stop:
        break;
    case calib_startup:
        {
            if(isSkinContact)
            {
                sendMsg("Contact on skin detected. Start calibration now.", MSG_INFO, true);
                posSignalAnalyzer->reset();
                forceSignalAnalyzer->reset();
				forceThreshold = FORCE_THRESHOLD;
                
                ref.xLink = contact.getLinkNumber()+TORSO_DOF; //controllerStatus.getContactLink();
                ref.x = contact.getCoP(); //controllerStatus.getXc_link();
				ref.xEnv = controllerStatus.getXc();

                firstInContactRobotPose = controllerStatus.getJointAng();
				firstContactPosition = contact.getCoP(); // controllerStatus.getXc_link();
				lastInContactRobotPose = controllerStatus.getJointAng();

                if(totTaxelConf >= MIN_TOTAL_CONFIDENCE)
                {
                    ref.f = SCRUB_FORCE_CONTROL;
                    // if the active taxels have no position estimate
                    if(norm(ref.x)==0.0)
                    {
                        sendMsg("START-UP ERROR!!! Active taxels have no position estimates!", MSG_WARNING);
                    }
                    else
                        changeCalibPhase(calib_moving);
                }
                else
                {
                    // control the contact point
                    ref.f = SCRUB_FORCE_CALIB;
                    sendMsg("The taxel position estimations are too uncertain so I am going to control the contact point.");
                    changeCalibPhase(calib_spiral);
                }

                sendMsg("Going to control point "+string(ref.x.toString(2).c_str())+" on link "+toString(ref.xLink));
                controller->setCtrlPnt(ref.xLink, ref.x);
				controller->setXd(ref.xEnv);
                controller->setFd(ref.f);
            }
            break;
        }
    case calib_end_eff:
        {
            if(explorationPhase!=move_back && !isSkinContact && Time::now()-isSkinContactTimestamp>CONTACT_LOST_TIMEOUT)
            {
                sendMsg("Move back because contact lost on the skin for more than "+toString(CONTACT_LOST_TIMEOUT)+" seconds.", MSG_INFO, true);
                ref.xEnv = initialXdEnv;
                explorationPhase = move_back;
                controller->setXd(ref.xEnv);
                controlImprovement->reset();
            }

            controlImprovement->addSample(controllerStatus.getX()-controllerStatus.getXd());
            if(controlImprovement->isImproving()==false)
            {
                if(totTaxelConf >= MIN_TOTAL_CONFIDENCE)
                {
                    sendMsg("Confidence of position estimation is high enough now!", MSG_INFO, true);
                    changeCalibPhase(calib_moving);
                    break;
                }

                if(explorationPhase==move_back)
                {
                    explorationPhase = move_away;
                    if(Rand::scalar(-1, 1)>-0.8)
                    {
                        // EXPLORE (rotate 45°)
                        Vector oldDir = versor(secondXdEnv-initialXdEnv);
                        Vector newDir = 0.5*(oldDir + versor(cross(oldDir, controllerStatus.getForce())));
                        secondXdEnv = initialXdEnv + norm(MOTION_STEP)*newDir;
                        ref.xEnv = secondXdEnv;
                        sendMsg("Moving forth to "+string(ref.xEnv.toString(2).c_str())+" (total taxel confidence "+toString(totTaxelConf)+")");
                    }
                    else
                    {
                        // EXPLOIT
                        ref.xEnv = leastConfidenceXdEnv.pos;
                        sendMsg("Moving to least confidence contact area: "+string(ref.xEnv.toString(2).c_str())+ 
                            " (avg taxel conf "+toString(leastConfidenceXdEnv.confidence)+")");
                    }
                    //calibrator->pauseCalibration();
                    sendMsg("Moving away", MSG_INFO, true);
                    sendMsg("Moving forth to "+string(secondXdEnv.toString(3).c_str())+" (total taxel confidence "+toString(totTaxelConf)+")");
                }
                else if(explorationPhase==move_away)
                {
                    explorationPhase = push_stronger;
                    ref.f = 1.2*SCRUB_FORCE_CALIB;
                    //calibrator->startCalibration();
                    sendMsg("Pushing harder", MSG_INFO, true);
                }
                else if(explorationPhase==push_stronger)
                {
                    explorationPhase = move_back;
                    ref.f = SCRUB_FORCE_CALIB;
                    ref.xEnv = initialXdEnv;
                    //calibrator->pauseCalibration();
                    sendMsg("Moving back", MSG_INFO, true);
                    sendMsg("Moving back to "+string(initialXdEnv.toString(3).c_str())+" (total taxel confidence "+toString(totTaxelConf)+")");
                }
                
                controller->setXd(ref.xEnv);
                controller->setFd(ref.f);
                controlImprovement->reset();
            }
            break;
        }
    case calib_spiral:
        {
            if(spiralPhase!=spiral_back && !isSkinContact && Time::now()-isSkinContactTimestamp>CONTACT_LOST_TIMEOUT)
            {
                sendMsg("Move back because contact lost on the skin for more than "+toString(CONTACT_LOST_TIMEOUT)+" seconds.", MSG_INFO, true);
                ref.xEnv = initialXdEnv;
                spiralPhase = spiral_back;
                currentMotionStep = SPIRAL_INITIAL_MOTION_STEP;
                currentMotionSubstep = 0.0;
                controller->setXd(ref.xEnv);
                controller->setQd(firstInContactRobotPose);
                controller->startControl(JPOS_CTRL);
                controlImprovement->reset();
            }

            controlImprovement->addSample(controllerStatus.getX()-controllerStatus.getXd());

            if(controlImprovement->isImproving()==false)
            {
                if(totTaxelConf >= MIN_TOTAL_CONFIDENCE)
                {
                    if(spiralPhase != spiral_back)
                    {
                        sendMsg("Confidence of position estimation is high enough now!", MSG_INFO, true);
                        spiralPhase = spiral_back;
                        controller->setQd(firstInContactRobotPose);
                        controller->startControl(JPOS_CTRL);
                        controlImprovement->reset();
                        break;
                    }
                    ref.xEnv = controllerStatus.getXc();
                    ref.x = contact.getCoP(); //controllerStatus.getXc_link();
                    controller->setCtrlPnt(ref.xLink, ref.x);
                    controller->setXd(ref.xEnv);
                    changeCalibPhase(calib_moving);
                    break;
                }

                if(spiralPhase == spiral_back)
                {
                    spiralPhase=spiral_forward;
                    controller->startControl(CONTACT_CTRL);
                    controlImprovement->reset();
                    isSkinContactTimestamp = Time::now();
                    break;
                }
                if(currentMotionSubstep>=currentMotionStep)
                {
                    currentMotionDirection = versor(cross(currentMotionDirection, controllerStatus.getForce()));
                    currentMotionStep += SPIRAL_STEP_INCREMENT;
                    currentMotionSubstep = 0.0;
                    sendMsg("Change direction to "+string(currentMotionDirection.toString(2).c_str()));
                }
                double step = min(SPIRAL_INITIAL_MOTION_STEP, currentMotionStep-currentMotionSubstep);
                currentMotionSubstep += step;
                //currentMotionDirection = rotateOntoPlane(currentMotionDirection, controllerStatus.getForce());
                ref.xEnv += step*currentMotionDirection;
                sendMsg("Spiral forward substep "+toString(step)+" (total taxel confidence "+toString(totTaxelConf)+")");
                sendMsg("Spiraling forward", MSG_INFO, true);
                
                controller->setXd(projectOnPlane(ref.xEnv, controllerStatus.getX(), controllerStatus.getForce()));
                controlImprovement->reset();
            }
            break;
        }
    case calib_moving:
        {
            bool taxelNotReached = true;
			for(unsigned int i=0; i< contact.getTaxelList().size(); i++)
			    if(contact.getTaxelList()[i] == ref.taxelId)
			        taxelNotReached = false;
	        if(taxelNotReached==false)
	        {
	            controller->setXd(ref.xEnv);
				changeCalibPhase(calib_pick_dest);
				sendMsg("Destination taxel reached.", MSG_INFO, true);
				break;
			}
			
            double controllerError = norm(controllerStatus.getX()-controllerStatus.getXd());
			if(controllerError < minError-MIN_POS_ERR_IMPROVEMENT)
			{
				minError = controllerError;
				if(firstAttempt)
				    errorMinimizationTimeout = int(STABILITY_TIME_WINDOW / (period*1e-3));
			    else
			        errorMinimizationTimeout = int(SECOND_ATTEMPT_STABILITY_TIME_WINDOW / (period*1e-3));
			}
			else{
				errorMinimizationTimeout--;
			}
            
            //if(errorMinimizationTimeout%int(1e3/period)==0)
            //   sendMsg("Error minimization timeout (in sec): "+toString(errorMinimizationTimeout*(period*1e-3)));

            if(!isSkinContact && Time::now()-isSkinContactTimestamp>CONTACT_LOST_TIMEOUT)
            {
                sendMsg("Contact on the skin lost", MSG_INFO, true);
                changeCalibPhase(calib_contact_lost);
            }
            else if(!isInContact && Time::now()-isInContactTimestamp>CONTACT_LOST_TIMEOUT)
            {
                sendMsg("Contact lost", MSG_INFO, true);
                changeCalibPhase(calib_contact_lost);
            }
			else if(errorMinimizationTimeout <= 0)
            {
				sendMsg("The controller is no longer able to reduce the position error that is: "+toString(controllerError));
				//sendMsg("The contact position error is: "+toString(norm(controllerStatus.getPos()-controllerStatus.getPos2())));
                
				lastInContactRobotPose = controllerStatus.getJointAng();
				//vector<unsigned int>::iterator iter = find(contact->getTaxelList().begin(),contact->getTaxelList().end(),ref.taxelId);
                //if(iter == contact->getTaxelList().end()){
				//    b2=true; sendMsg("Taxel not active"); 
			    //}
			    
				bool b1 = (norm(controllerStatus.getX()-controllerStatus.getXc()) > MIN_POSITION_ERROR);
                bool b3 = (norm(controllerStatus.getXc()-ref.xEnv)<MAX_ENV_CONTACT_DISTANCE);
                

				if(b1 && taxelNotReached && b3 && ref.taxelId != -1){
					if(minError < overallMinError){
					    firstAttempt = false;
					    sendMsg("Trying again", MSG_INFO, true);
						//sendMsg("Resetting the xEnv to the actual contact point position");
						overallMinError = minError;
						//sendMsg("New overall min error is "+toString(minError));
						minError = DBL_MAX;
						errorMinimizationTimeout = int(SECOND_ATTEMPT_STABILITY_TIME_WINDOW / (period*1e-3));
						posSignalAnalyzer->reset();
						controller->setXd(controllerStatus.getXc());
					}
					else{
					    //sendMsg("Picking a new desired position because the last min error is "+toString(minError) + " and the overall min error is "+toString(overallMinError));
						controller->setXd(ref.xEnv);
						changeCalibPhase(calib_pick_dest);
					}
				}
				else{
					if(ref.taxelId != -1){
						if(taxelNotReached)
							sendMsg("The destination taxel "+toString(ref.taxelId)+" has not been reached.");
						else
							sendMsg("The destination taxel has been reached.", MSG_INFO, true);
						//sendMsg("The actual contact point distance is "+toString(norm(controllerStatus.getPos2()-ref.xEnv)));
					}
					controller->setXd(ref.xEnv);
					changeCalibPhase(calib_pick_dest);
				}
            }
            break;
        }
    case calib_pick_dest:
        {
            // Create a list of all the eligible taxels
            firstAttempt = true;
			sendMsg("Picking new taxel", MSG_INFO, true);
            set<unsigned int> eligibleTaxels;
            vector<unsigned int> activeTaxelList = contact.getTaxelList();
            if(activeTaxelList.size()==0)
            {
                sendMsg("Error in calib_pick_dest: no active taxels.", MSG_WARNING);
                changeCalibPhase(calib_contact_lost);
                break;
            }
            //sendMsg("Active taxel list size: "+toString(activeTaxelList.size()));   
            for(unsigned int i=0; i<contact.getActiveTaxels(); i++)    // for each active taxel
            {
                unsigned int taxelId = activeTaxelList[i];
                list<unsigned int> *neigh = &(neighborsXtaxel[taxelId]);
                for(list<unsigned int>::const_iterator it=neigh->begin(); it!=neigh->end(); it++)   // for each neighbor
                {
                    // add the taxel to the list (if it is not already in the list)
                    eligibleTaxels.insert(*it);
                }
            }
            //sendMsg(toString(activeTaxelList.size())+" taxels in contact. "+toString(eligibleTaxels.size())+" eligible taxels.");

            // pick a taxel from the list with uniform probability
			set<unsigned int>::iterator it;
			double min_score = DBL_MAX;
			vector<int> same_score;
			for(it = eligibleTaxels.begin(); it != eligibleTaxels.end(); it++){
				double rel = skinClient->getPoseConfidence(skinPart,*(it));
				double score = TAXEL_SELECTION_WEIGHT*rel + (1.0-TAXEL_SELECTION_WEIGHT)*(taxelVisitCount[*(it)]/maxTaxelVisitCount);
				
				int triangleId = (*it)/TAXELS_PER_TRIANGLE;
				bool isTaxelEligible = false;
				for(unsigned int i=0; i<ELIGIBLE_TRIANGLES.size(); i++)
				    if(triangleId==ELIGIBLE_TRIANGLES[i])
				        isTaxelEligible = true;
		        if(isTaxelEligible==false)
		            score = 1.1;
		        //printf("Taxel %d belongs to triangle %d and isTaxelEligile=%s\n", *it, triangleId, isTaxelEligible?"true":"false");
		        
				if(score < min_score && norm(taxelPos[*it])!=0.0){
					min_score = score;
					same_score.clear();
					same_score.push_back(*(it));
				}
				else if(score == min_score && norm(taxelPos[*it])!=0.0)
					same_score.push_back(*(it));
			}
			if(same_score.size() == 0){
				sendMsg("Cannot pick a taxel, going to restore");
				changeCalibPhase(calib_restore);
				break;
			}
			ref.taxelId = same_score[(unsigned int)Rand::scalar(0, same_score.size()-1)];
			taxelVisitCount[ref.taxelId]++;
			if(taxelVisitCount[ref.taxelId] > maxTaxelVisitCount)
				maxTaxelVisitCount = taxelVisitCount[ref.taxelId];
            // set the new reference position for the controller
            ref.x = taxelPos[ref.taxelId];
			ref.xLink = contact.getLinkNumber()+TORSO_DOF; 
            ref.f = SCRUB_FORCE_CONTROL;
			isFirstTry=true;
            controller->setCtrlPnt(ref.xLink, ref.x);
			posSignalAnalyzer->reset();
			forceSignalAnalyzer->reset();   
            sendMsg("New destination is taxel "+toString(ref.taxelId)+" in position "+ref.x.toString(2).c_str()+" on link "+toString(ref.xLink));

            changeCalibPhase(calib_moving);
            break;
        }
    case calib_contact_lost:
        {
            if(isSkinContact)
            {
                sendMsg("Contact on skin restored!", MSG_INFO, true);
				forceThreshold = FORCE_THRESHOLD;
                changeCalibPhase(calib_pick_dest);
            }
            else if(isPosStable && isForceStable) //Change to stable Joint angles?
            {
                sendMsg("The last recorded in-contact position has been restored but the robot is not in contact on the skin. The initial robot position will be restored.");
                changeCalibPhase(calib_restore);
            }
            break;
        }
	case calib_restore:
		{
			if(isPosStable)
            {
                if(!isRestored)
                {
				    isRestored = true;
                    sendMsg("Initial Position Restored, moving to the first in-contact position!");
				    controller->setQd(firstInContactRobotPose);
				    posSignalAnalyzer->reset();
				    forceSignalAnalyzer->reset();
                    //Bottle zeroFT;
				    //zeroFT.addInt(0);
				    //port_ft_rpc->write(zeroFT);
				    //sendMsg("FT sensor re-calibrated!");
                }
			    else 
                {
                    if(isSkinContact)
                    {
				        sendMsg("Initial in-contact Position Restored!");
				        isFirstTry = false;
                        // TODO: check whether ref.x = [0 0 0]
				        ref.x = contact.getCoP();
				        changeCalibPhase(calib_moving);
			        }
			        else
                    {
				        forceThreshold = FORCE_THRESHOLD;
                        ref.x = firstContactPosition;
			        }
                    posSignalAnalyzer->reset();
				    forceSignalAnalyzer->reset();
                    controller->setXd(ref.xEnv);
                    controller->setCtrlPnt(ref.xLink, ref.x);
                    controller->startControl(CONTACT_CTRL);
                }
            }
            break;
		}
    default:    
        sendMsg("Unknown calibration phase in updateCTrlRef", MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::sanityCheck() throw()
{

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void calibrationPlannerThread::sendMonitorData() throw()
{
    skinDestination.zero();
    if(ref.taxelId>=0 && (unsigned int)ref.taxelId<skinDestination.size())
        skinDestination[ref.taxelId] = 100.0;
    port_skin_dest->prepare() = skinDestination;
    port_skin_dest->write();

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
void calibrationPlannerThread::sendMsg(const string &msg, const MsgType &type, bool toSend) throw()
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
