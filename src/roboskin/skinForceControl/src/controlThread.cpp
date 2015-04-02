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

#include "iCub/skinForceControl/controlThread.h"
#include "iCub/skinForceControl/util.h"
#include "iCub/skinDynLib/common.h"
#include <iCub/ctrl/math.h>

#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <iostream>

#define TEST_TORQUE_CTRL    // use customized torque controller

using namespace yarp::os;
using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;
using namespace iCub::iKin;
using namespace iCub::ctrl;

IController::IController() throw(){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
IController::IController(string _name, string robotName, int _period, BodyPart _bodyPart, VerbosityLevel _verbose) throw()
: name(_name), period(_period), bodyPart(_bodyPart), verbose(_verbose){
    //---------------------DEVICES--------------------------//
    robotInt = new RobotInterfaces(name.c_str(), robotName.c_str(), bodyPart, TORSO);

    //----------- INIT VARIABLES----------------//
    ctrlLaw			    = NO_CONTROL;
    cmdMode             = SIMULATION;
    ctrlLawChanged      = true;
    controlForceNorm    = DEFAULT_CONTROL_FORCE_NORM;

    robotChain = new RobotArm(robotInt, bodyPart);
    dof = robotChain->getN();
    robot.ddq = robot.dq = robot.q = robot.torques = zeros(dof);
    fSmooth = robot.wrench = zeros(6);
    robot.contactNormal = cat(0, 0, 1);
    robot.contactPressure = 0.0;
    robot.w0 = robot.dw0 = robot.d2p0 = zeros(3);
    robot.d2p0[2] = GRAV_ACC;   // the gravity has to be expressed w.r.t. the root reference frame (not the 0th ref frame!)
                                // note that the vector d2p0 is equal and opposite to the gravity
    robot.activeJoints.resize(dof, 0.0);     // unblock shoulder and elbow (torso and wrist are blocked)
    robot.activeJoints.setSubvector(3, cat(1.0, 1.0, 0.0, 1.0));

    ctrlRef.dqMax = DEFAULT_DQ_MAX;
    ctrlRef.pinvDamp = DEFAULT_PINV_DAMP;
	ctrlRef.torques = commandRef = zeros(dof);
    qd = ctrlRef.q = DEFAULT_QD;
    ctrlRef.wrench = fd = DEFAULT_FD;
    ctrlRef.linkNumber = ctrlRef.linkNumber2 = dof-1;
    ctrlRef.ctrlPointRef = ctrlRef.ctrlPoint = ctrlRef.ctrlPoint2 = zeros(3); //cat(0.0, 0.06, -0.04);
    ctrlRef.dx = ctrlRef.ddx = zeros(3);
    if(bodyPart==LEFT_ARM)
        x = x2 = DEFAULT_XD_LEFT;
    else
        x = x2 = DEFAULT_XD_RIGHT;
    ctrlRef.x = ctrlRef.xRef = x.subVector(0,2); //cat(-0.17, 0.2, 0.01);
    J.resize(6, dof);
    dJ.resize(6, dof);
    k_comp = zeros(dof);
    
    lpFilter_f      = new FirstOrderLowPassFilter(DEFAULT_FC_F, period/1000.0, zeros(6));
    lpFilter_fSmooth= new FirstOrderLowPassFilter(DEFAULT_FC_F_SMOOTH, period/1000.0, zeros(6));
    lpFilter_ref    = new FirstOrderLowPassFilter(DEFAULT_FC_REF, period/1000.0, zeros(dof));
    lpFilter_fd     = new FirstOrderLowPassFilter(DEFAULT_FC_FD, period/1000.0, zeros(6));
    lpFilter_CoP    = new FirstOrderLowPassFilter(DEFAULT_FC_COP, period/1000.0, zeros(3));
    trajGen_xd      = new minJerkRefGen(3, period*1e-3, DEFAULT_TRAJ_TIME_XD);
    trajGen_ctrlPnt = new minJerkTrajGen(3, period*1e-3, DEFAULT_TRAJ_TIME_CTRL_PNT_D);
    trajGen_qd      = new minJerkRefGen(dof, period*1e-3, DEFAULT_TRAJ_TIME_QD);
    dqEst           = new AWLinEstimator(16, 1.0);
    ddqEst          = new AWQuadEstimator(25, 1.0);
    feasibility     = 1.0;
    contact         = new skinContact(bodyPart, SKIN_PART_UNKNOWN, dof-1, zeros(3), zeros(3), 0, 0.0);

    robotChain->setAng(robot.q);
    robotChain->setDAng(zeros(dof));
	robotChain->setD2Ang(zeros(dof));
    robotChain->prepareNewtonEuler();
    robotChain->initNewtonEuler(robot.w0, robot.dw0, robot.d2p0, zeros(3), zeros(3));
    
    // *** INITIALIZE CONTROL LAWS ***
    ctrlLaws[TORQUE_CTRL] = new TorqueControlLaw(robotChain, DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[FLOAT_CTRL] = new FloatControlLaw(robotChain, DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[JPOS_CTRL] = new JPositionControlLaw(robotChain, period, DEFAULT_JPOS_KP, DEFAULT_JPOS_KI, DEFAULT_JPOS_KD, 
                                                DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[POS_CTRL] = new PositionControlLaw(robotChain, period, DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD, 
                                                DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[FORCE_CTRL] = new ForceControlLaw(robotChain, period, DEFAULT_FORCE_KP, DEFAULT_FORCE_KI, DEFAULT_FORCE_KD, 
                                                DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR, false);    // disable moment control
    ctrlLaws[PARAL_CTRL] = new ParallelControlLaw(robotChain, period, DEFAULT_PARAL_KP, DEFAULT_PARAL_KI, DEFAULT_PARAL_KD,
                                                DEFAULT_PARAL_KF, DEFAULT_PARAL_KV, DEFAULT_KJ, 
                                                DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[DYN_PARAL_CTRL] = new DynParallelControlLaw(robotChain, period, DEFAULT_DPARAL_KP, DEFAULT_PARAL_KI, 
                                                DEFAULT_PARAL_KD, DEFAULT_PARAL_KF, DEFAULT_DPARAL_KV, DEFAULT_KJ, 
                                                DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[CONTACT_CTRL] = new ContactControlLaw(robotChain, period, DEFAULT_CONTACT_KP, DEFAULT_CONTACT_KI, DEFAULT_CONTACT_KD,
                                                DEFAULT_CONTACT_KF, DEFAULT_CONTACT_KV, DEFAULT_CONTACT_TRAJ_TIME,
                                                DEFAULT_CONTACT_FORCE_PRIORITY, DEFAULT_CONTACT_KC,
                                                DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);
    ctrlLaws[PRESS_CTRL] = new PressureControlLaw(robotChain, period, DEFAULT_PRESS_KP);
    ctrlLaws[SAFE_REACH_RIGID_CTRL] = new SafeReachRigidControlLaw(robotChain, period, DEFAULT_REACH_KP, DEFAULT_REACH_KV, 
                                                DEFAULT_CONTACT_KF, DEFAULT_REACH_KD, DEFAULT_REACH_KP_REST, FORCE_THRESHOLD,
                                                DEFAULT_KJ, DEFAULT_ACTIVATION_THR, DEFAULT_DEACTIVATION_THR);

    // *** POPULATE COMMAND LIST ***
    commandList.assign(IControllerCommand_s, IControllerCommand_s+IControllerCommandSize);
    commandDesc.assign(IControllerCommand_desc, IControllerCommand_desc+IControllerCommandSize);
    mutex.setName("IControllerMutex");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
controlThread::controlThread(string _moduleName, string robotName, int _period, string _wholeBodyName, 
                             BodyPart _bodyPart, VerbosityLevel _verbose) throw()
: RateThread(_period), wholeBodyName(_wholeBodyName), IController(_moduleName, robotName, _period, _bodyPart, _verbose)
{
    //---------------------PORTS-------------------------//	
    string slash = "/";
	port_torques        = new BufferedPort<Vector>;
    port_monitor        = new BufferedPort<Vector>;
    port_ext_contacts   = new BufferedPort<skinContactList>;
    port_ext_ft_sens    = new BufferedPort<Vector>;
	if(!port_torques->open((slash+name+"/torques:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the torques:o port");    
	if(!port_monitor->open((slash+name+"/monitor:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the monitor:o port");
    if(!port_ext_contacts->open((slash+name+"/ext_contacts:i").c_str()))
        thread_status.addErrMsg("It was not possible to open the ext_contacts:i port");
    if(!port_ext_ft_sens->open((slash+name+"/ext_ft_sens:i").c_str()))
        thread_status.addErrMsg("It was not possible to open the ext_ft_sens:i port");

#ifdef TEST_TORQUE_CTRL
    if(!torqueCtrlTestPort.open((slash+name+"/torqueCtrlTest:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the torqueCtrlTest:o port");
	Network::connect(torqueCtrlTestPort.getName().c_str(), "/torqueCtrlTest/rpc");
#endif
    
	//----------- INIT VARIABLES----------------//
    durations.resize(12, 0.0);
    timestamps.resize(12, 0.0);
#ifdef PREDICT_ERROR
    e_w = e_p = e_wd = zeros(3);
    e_tau = zeros(3);
#endif

    // *** ADD COMMANDS TO COMMAND LIST ***
    int cmdSize = IControlThreadCommandSize-IControllerCommandSize;
    commandList.insert(commandList.end(), IControlThreadCommand_s, IControlThreadCommand_s+cmdSize);
    commandDesc.insert(commandDesc.end(), IControlThreadCommand_desc, IControlThreadCommand_desc+cmdSize);
    mutex.setName("CtrlThreadMutex");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool controlThread::threadInit(){
    DSemaphore::registerThread("CtrlThread");
    Status s;
    // CHECK THE CONTROL LAWS
    for(map<ControlLaw,IControlLaw*>::const_iterator it=ctrlLaws.begin(); it!=ctrlLaws.end(); it++){
        if(it->second){
            Status cls=it->second->getStatus();
            if(cls==false){
                sendMsg("Ctrl law "+it->second->getName()+" initialization failed: "+cls.errstr, MSG_ERROR);
                return false;
            }
        }
    }

    // CONNECT TO THE ROBOT
    if(!robotInt->init()){
        sendMsg("Robot interfaces initialization failed", MSG_ERROR);
        return false;
    }

    if(!(s = robotChain->init())){
        sendMsg(s.toString(), MSG_ERROR);
        return false;
    }

    // *** ENCODERS ***
    sendMsg("Trying to read arm encoders...");
    robotChain->readAng(robot.q, true);
    
    timePre = encTimestamp = torqueTimestamp = extContTimestamp = Time::now();
    
#ifndef _SIMULATION
    // check if wholeBody is running
	string wbContPortName = "/"+wholeBodyName+"/contacts:o";
    while(thread_status && !Network::exists(wbContPortName.c_str())){
        sendMsg("Waiting for wholeBodyDynamics (port: "+wbContPortName+")...");
		Time::delay(1.0);
	}
    if(!thread_status) return false;
    // try to connect to the wholeBodyDynamics output contact port
    sendMsg("wholeBodyDynamics contact port exists! Trying to connect...");
    if(!Network::connect(wbContPortName.c_str(), port_ext_contacts->getName().c_str())){
        sendMsg("Connection failed. Stopping the thread", MSG_ERROR);
        return false;
    }
    printf("connection succeeded!\nTrying to read contacts...");
    skinContactList contList = *(port_ext_contacts->read(true));
    printf("read succeeded!\n");
    if(!contList.empty()){
        robot.wrench = contList[0].getForceMoment();
    }

    string wbExtFtSensPortName = "/"+wholeBodyName+"/"+BodyPart_s[bodyPart]+"/ext_ft_sens:o";
    while(thread_status && !Network::exists(wbExtFtSensPortName.c_str())){
        sendMsg("Waiting for wholeBodyDynamics (port: "+wbExtFtSensPortName+")...");
		Time::delay(1.0);
	}
    if(!thread_status) return false;
    // try to connect to the wholeBodyDynamics output contact port
    sendMsg("wholeBodyDynamics external ft sensor port exists! Trying to connect...");
    if(!Network::connect(wbExtFtSensPortName.c_str(), port_ext_ft_sens->getName().c_str())){
        sendMsg("Connection failed. Stopping the thread", MSG_ERROR);
        return false;
    }
    printf("connection succeeded!\nTrying to read ext ft sens...");
    robot.extFtSens = *(port_ext_ft_sens->read(true));
    printf("read succeeded!\n");

    timePre = encTimestamp = torqueTimestamp = extContTimestamp = Time::now();
#endif

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::threadRelease(){
    sendMsg("Control thread release.\n");
    Status s;
	if (robotInt->icmd[bodyPart]){
        sendMsg( "Setting position control mode for all the joints.\n");
        s=robotChain->setPositionMode();
        if(!s)
            sendMsg(s.errstr, MSG_ERROR);
	}
    if (robotInt->itrq[bodyPart]){
        sendMsg( "Setting zero reference torque for all the joints.\n");
        s=robotChain->setRefTorques(zeros(dof));
        if(!s)
            sendMsg(s.errstr, MSG_ERROR);
	}

    if(trajGen_qd)      delete trajGen_qd;
    if(trajGen_xd)      delete trajGen_xd;
    if(trajGen_ctrlPnt) delete trajGen_ctrlPnt;
    if(lpFilter_f)      delete lpFilter_f;
    if(lpFilter_fSmooth)delete lpFilter_fSmooth;
    if(lpFilter_ref)    delete lpFilter_ref;
    if(lpFilter_fd)     delete lpFilter_fd;
    if(lpFilter_CoP)    delete lpFilter_CoP;
    if(dqEst)           delete dqEst;
    if(ddqEst)          delete ddqEst;
    for(unsigned int i=0; i< (unsigned int)CONTROL_LAW_SIZE; i++) 
        if(ctrlLaws[(ControlLaw)i]) 
            delete ctrlLaws[(ControlLaw)i];
    
	Time::delay(0.5);

    if(port_torques)        { port_torques->interrupt(); port_torques->close(); delete port_torques;  port_torques = 0;}
    if(port_monitor)        { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
    if(port_ext_contacts)   { port_ext_contacts->interrupt(); port_ext_contacts->close(); delete port_ext_contacts;  port_ext_contacts = 0;}
    if(port_ext_ft_sens)    { port_ext_ft_sens->interrupt(); port_ext_ft_sens->close(); delete port_ext_ft_sens;  port_ext_ft_sens = 0;}
    if(robotInt)            { robotInt->close(); delete robotInt; robotInt=0;}
    if(robotChain)          {delete robotChain; robotChain=0;}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::run(){
    // check everything is working fine
    sanityCheck();
    
    mutex.wait();
    {
        if( !thread_status )
            return mutex.post();

        // CONTROL LOOP
        if(ctrlLawChanged){
            // if the ctrl mode has changed, initialize the new ctrl mode
            sendMsg("Control mode just changed to "+ControlLaw_desc[ctrlLaw], MSG_INFO);
            initCtrlLaw(ctrlLaw);
            ctrlLawChanged = false;
        }
        updateRobotStatus();
        feasibility = computeCtrlRef(ctrlLaw);  // 0.99*feasibility + 0.01*computeCtrlRef(ctrlLaw);
        sendCtrlRef(ctrlLaw);
        sendMonitorData(ctrlLaw);
    }
    mutex.post();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::updateRobotStatus() throw(){
    // *** READ JOINT ENCODERS ***
    Status s = robotChain->readAng(robot.q);
    if(s){
        AWPolyElement el;
        el.time = encTimestamp = Time::now();
        el.data = robot.q;
        robot.dq = dqEst->estimate(el);
        robot.ddq = ddqEst->estimate(el);
        robotChain->setAng(robot.q*CTRL_DEG2RAD);
    }else
        sendMsg(s.errstr, MSG_WARNING);

    // *** READ JOINT TORQUES *** 
    if(s=robotChain->readTorques(robot.torques))
        torqueTimestamp = Time::now();
    else
        sendMsg(s.errstr, MSG_WARNING);

    // *** READ EXTERNAL WRENCH AT THE F/T SENSOR
    Vector* temp = port_ext_ft_sens->read(false);
    if(temp)
        robot.extFtSens = *temp;

    // *** READ EXTERNAL CONTACTS ***
    skinContactList* contList = port_ext_contacts->read(false);
    if(contList)
    {
        // take the contact that activates the most taxels
        extContTimestamp = Time::now();
        unsigned int maxTaxels = 0;
        partContList.clear();
        for(skinContactList::iterator it=contList->begin(); it!=contList->end(); it++){
            if(it->getBodyPart() == bodyPart){
                partContList.push_back(*it);
                if(it->getActiveTaxels() >= maxTaxels){
                    maxTaxels = it->getActiveTaxels();
                    contact = &(*it);
                }
            }
        }
        robot.contactList = partContList;

        // if more than one contact
        if(partContList.size()>1){
            // project external wrench at F/T sens to contact point
            Matrix H_c_s = eye(4,4);               // from contact to contact link
            H_c_s.setSubcol(contact->getCoP(), 0, 3);
            H_c_s *= robotChain->getH_i_j(contact->getLinkNumber()+TORSO_DOF, robotChain->getSensorLinkIndex()); // from contact link to sensor link
            H_c_s *= robotChain->getHSens();       // from sensor link to sensor
            contact->setForceMoment(wrenchProjectionMatrix(H_c_s) * (-1.0*robot.extFtSens));
        }
#ifdef _DEBUG
        else{
            // TEST project external wrench at F/T sens to contact point
            Matrix H_c_s = eye(4,4);               // from contact to contact link
            H_c_s.setSubcol(contact->getCoP(), 0, 3);
            H_c_s *= robotChain->getH_i_j(contact->getLinkNumber()+TORSO_DOF, robotChain->getSensorLinkIndex()); // from contact link to sensor link
            H_c_s *= robotChain->getHSens();       // from sensor link to sensor
            Vector wrenchProj = wrenchProjectionMatrix(H_c_s) * (-1.0*robot.extFtSens);
            //if(norm(wrenchProj-contact->getForceMoment()) > 1e-2){
            //    /*printf("Error in projecting extFtSens. \nContact wrench: %s != \nFt sens proj: %s\n", 
            //        contact->getForceMoment().toString(2).c_str(), wrenchProj.toString(2).c_str());*/
            //    printf("Projection error: %f\n", norm(wrenchProj-contact->getForceMoment()));
            //}
        }
#endif
        contactToSend = *contact;
        // update force ctlr point
        ctrlRef.linkNumber2 = contact->getLinkNumber()+TORSO_DOF;
        ctrlRef.ctrlPoint2 = lpFilter_CoP->filt(contact->getCoP());     // CoP in link reference frame
    }

    // contact point position
    H_r_c2 = robotChain->getH(ctrlRef.linkNumber2);                  // rototranslation from root to contact point link
    R_r_c2 = H_r_c2.submatrix(0,2,0,2);                               // rotation from root to contact point link
    x2 = (H_r_c2 * cat(contact->getCoP(), 1.0)).subVector(0,2);     // project contact point in root frame

    // control point position
    H_r_c = robotChain->getH(ctrlRef.linkNumber);   // rototranslation from root to pos ctrl point link
    x = (H_r_c*cat(ctrlRef.ctrlPoint, 1.0)).subVector(0,2);

    // project contact force in root frame
    if(contList)
        contact->setForceMoment( projectSixDimVector(contact->getForceMoment(), R_r_c2));
    // TODO: check whether the contact link has changed, in that case disable the low pass filter
    robot.wrench = lpFilter_f->filt(contact->getForceMoment());
    robot.wrenchSmooth = fSmooth = lpFilter_fSmooth->filt(contact->getForceMoment());
    robot.contactNormal = contact->getNormalDir();
    robot.contactPressure = contact->getPressure();

    // MIN JERK TRAJ GEN (cartesian space)
    trajGen_xd->computeNextValues(x, ctrlRef.xRef);
    ctrlRef.x = trajGen_xd->getPos();
    ctrlRef.dx = trajGen_xd->getVel();
    ctrlRef.ddx = trajGen_xd->getAcc();

    // MIN JERK TRAJ GEN (control point)
    trajGen_ctrlPnt->computeNextValues(ctrlRef.ctrlPointRef);
    ctrlRef.ctrlPoint = trajGen_ctrlPnt->getPos();
    

    // *** POSITION ERROR SATURATION ***
    Vector posErr = ctrlRef.x - x;    // pos err in root frame
    double posErrNorm = norm(posErr);
    if(posErrNorm > MAX_POS_ERROR)
        ctrlRef.x = x + posErr*MAX_POS_ERROR/posErrNorm;

    // MIN JERK TRAJ GEN (joint space)
    trajGen_qd->computeNextValues(robot.q, qd);
    ctrlRef.q = trajGen_qd->getPos();
    ctrlRef.dq = trajGen_qd->getVel();
    ctrlRef.ddq = trajGen_qd->getAcc();

    // control point velocity and acceleration
    Vector dqRad = CTRL_DEG2RAD*robot.dq;
    R_r_c = H_r_c.submatrix(0,2,0,2);
    Vector ctrlPnt_r = R_r_c*ctrlRef.ctrlPoint;
    J.zero(); dJ.zero();
    J.setSubmatrix(robotChain->GeoJacobian(ctrlRef.linkNumber), 0, 0);
    dJ.setSubmatrix(robotChain->DJacobian(ctrlRef.linkNumber, dqRad), 0, 0);
    contactPointJacobian(ctrlPnt_r, J);
    contactPointJacobian(ctrlPnt_r, dJ);
    dx = (J*dqRad).subVector(0,2);
    ddx = (J*(CTRL_DEG2RAD*robot.ddq) + dJ*dqRad).subVector(0,2);
    
    // LOW PASS FILTERS
    Vector fdRef = fd;
    // turn the desired force towards the direction of the measured force
    if(controlForceNorm)
    {
        Vector f = robot.wrench.subVector(0,2);
        double norm_f = norm(f);
        double norm_fd = norm(fd);
        if(norm_f >= 0.1) //max<double>(0.5*norm_fd, FORCE_THRESHOLD))
            fdRef.setSubvector(0, f*norm_fd/norm_f);
    }
    ctrlRef.wrenchRef = fdRef;
    ctrlRef.wrench = lpFilter_fd->filt(fdRef);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::initCtrlLaw(const ControlLaw &cl) throw(){
	if(cl==NO_CONTROL){
        if(cmdMode == REAL)
		    robotChain->setPositionMode();
		return;
	}

    ctrlLaws[cl]->reset(ctrlRef, robot);
    ControlMode cm = ctrlLaws[cl]->getControlMode();
    if(cmdMode == REAL)
        for(int i=0;i<dof;i++)
            if(robot.activeJoints[i]==1.0)  // if the joint is not blocked
                robotChain->setCtrlMode(cm, i);
    
    // reset filters
    switch(cm){
        case TORQUE_CTRL_MODE:  lpFilter_ref->init(robot.torques);  break;
        case VEL_CTRL_MODE:     lpFilter_ref->init(robot.dq);       break;
        case POS_CTRL_MODE:     lpFilter_ref->init(robot.q);        break;
    }
    lpFilter_fd->init(robot.wrench);
    trajGen_qd->init(robot.q);
    trajGen_xd->init(x);

	return;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double controlThread::computeCtrlRef(const ControlLaw &cl) throw(){
    if(cl==NO_CONTROL)
        return 1.0;
    double f;
    Status res = ctrlLaws[cl]->compute(ctrlRef, robot, commandRef, f);
    if(!res)
        sendMsg(res.errstr, MSG_WARNING);
    commandRef += k_comp*(commandRef-robot.torques);
    return f;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::sendCtrlRef(const ControlLaw &cl) throw(){  
	// command vector low pass filter
	commandRef = lpFilter_ref->filt(commandRef);

    if(cl!=NO_CONTROL && cmdMode==REAL){
        ControlMode cm = ctrlLaws[cl]->getControlMode();
        for(int i=0;i<dof;i++)
            if(robot.activeJoints[i]==1.0)                      // if the joint is not blocked
            {

#ifndef TEST_TORQUE_CTRL
                robotChain->setRef(cm, i, commandRef[i]);       // send the ref commands to the motors
#else               
                Bottle b, resp;
                stringstream setTaodCommand;
                setTaodCommand<< "set taod ";
                setTaodCommand<< commandRef[i];
                printf("%s", setTaodCommand.str().c_str());
		        b.fromString(setTaodCommand.str().c_str());
		        torqueCtrlTestPort.write(b,resp);
#endif
            }
    }

    // write command references on the output port
    port_torques->prepare() = commandRef;
	port_torques->write();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::sanityCheck() throw(){
    double now = Time::now();
    if(now-encTimestamp> PORT_TIMEOUT){
        stringstream ss;
        ss<< "controlThread can't read encoders for "<< now-encTimestamp<< " sec";
        sendMsg (ss.str(), MSG_ERROR);
    }
    if(now-torqueTimestamp> PORT_TIMEOUT){
        stringstream ss;
        ss<< "controlThread can't read torques for "<< now-torqueTimestamp<< " sec";
        sendMsg (ss.str(), MSG_ERROR);
    }

#ifndef _SIMULATION
    if(now-extContTimestamp > PORT_TIMEOUT){
        stringstream ss;
        ss<< "controlThread can't read from "<< port_ext_contacts->getName()<< " for "<< now-extContTimestamp<< " sec";
        sendMsg (ss.str(), MSG_ERROR);
    }
#endif

    //check if interfaces are still up (icubinterface running)  
	if (robotInt->icmd[bodyPart] == 0){
        sendMsg("ControlLaw interface already closed, stopping the controller.", MSG_ERROR);    
    }
	if (robotInt->itrq[bodyPart] == 0){
        sendMsg("TorqueControl interface already closed, stopping the controller.", MSG_ERROR);  
    }
    if (robotInt->ienc[bodyPart] == 0){
        sendMsg("Encoder interface closed, stopping the controller.", MSG_ERROR);    
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::sendMonitorData(const ControlLaw &cl) throw(){
    // if nobody is reading do not write
    if(port_monitor->getOutputCount()==0)
        return;

    data.setContact(contactToSend);
    if(ctrlLaws[cl])
        data.setExtraData(ctrlLaws[cl]->getMonitorData());

    data.setTorques(robot.torques.subVector(0,7));
    data.setRefCommand(commandRef.subVector(0,7));

    data.setForce(robot.wrench.subVector(0,2));
    data.setForceNorm(norm(robot.wrench.subVector(0,2)));
    data.setRefForce(ctrlRef.wrench.subVector(0,2));

    data.setXc(x2);             // contact point, in root ref frame
    data.setX(x);               // position control Point, in root ref frame
    data.setDx(dx);             // end effector vel
    data.setDdx(ddx);           // end effector acc

    data.setJointAng(robot.q);
    data.setJointAngRef(ctrlRef.q);
    data.setJointVel(robot.dq);
    data.setJointAcc(robot.ddq);

    data.setXd(ctrlRef.xRef);               // destination position of ctrlPoint (root ref frame)
    data.setRefX(ctrlRef.x);                // current reference position
    data.setRefDx(ctrlRef.dx);
    data.setRefDdx(ctrlRef.ddx);

    data.setFeasibility(feasibility);
    data.setContactNumber(partContList.size());

    Vector pwm(16+TORSO_DOF);
    robotInt->iopl[bodyPart]->getOutputs(pwm.data()+TORSO_DOF);
    data.setPwm(pwm.subVector(0,dof-1));
    data.setPwmNorm(norm(pwm.subVector(3,6)));  // only shoulder+elbow DOFs

    port_monitor->prepare() = data.getData();
    port_monitor->write();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlThread::sendMsg(const string &msg, const MsgType &type) throw(){
    if(type==MSG_ERROR){
        thread_status.addErrMsg(msg);
    }
    //printf("\n");
    printf("%s: %s\n", MsgType_s[type].c_str(), msg.c_str());
    //Bottle& b = infoPort.prepare();
    //b.clear();
    //b.addString(msg.c_str());
    //infoPort.write();
}
