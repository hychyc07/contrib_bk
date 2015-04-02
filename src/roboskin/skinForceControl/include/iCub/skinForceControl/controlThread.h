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

#ifndef CTRL_THREAD
#define CTRL_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/YarpNameSpace.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "iCub/skinForceControl/controlConstants.h"
#include "iCub/skinForceControl/controlLaws.h"
#include "iCub/skinForceControl/utilRobotics.h"
#include "iCub/skinForceControl/robotArm.h"
#include "iCub/skinForceControl/skinForceControlClient.h"

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

namespace skinForceControl
{

const std::string IControllerCommand_s[]  = {
    "stop",                 "float ctrl",           "jpos ctrl",            "pos ctrl", 
    "torque ctrl",          "force ctrl",           "paral ctrl",           "dparal ctrl",
    "cont ctrl",            "press ctrl",           "reach ctrl",           "get ctrl",
    "set qd",               "get qd",
    "set ctrlPnt",          "get ctrlPnt",          "get x",
    "set ctrlLink",         "get ctrlLink",
    "set xd",               "get xd",
    "set taud",             "set taudj",            "get taud",             "get tau",
    "set fd",               "get fd",               "get f",
    "set alphaf",           "get alphaf",           "set alphafd",          "get alphafd",
    "set alphaTaod",        "get alphaTaod",        "set traj time",        "get traj time",
    "sim on",               "sim off",
    "block joint",          "unblock joint",        "is active",
    "set pinv damp",        "get pinv damp",        "set kcomp",            "get kcomp"
};

// the order in IControllerCommand_desc must correspond to the order in IControllerCommand_s
const std::string IControllerCommand_desc[]  = {
    "stop controller",
	"set control law to float (gravity compensation)", 
	"set control law to joint position", 
	"set control law to cartesian position", 
    "set control law to joint torque",
	"set control law to cartesian force", 
	"set control law to parallel control",
    "set control law to dynamic parallel control",
    "set control law to contact control",
    "set control law to pressure control",
    "set control law to safe reach control",
    "get the name of the current control law",
	"set desired joint positions (used only in joint position control)",
    "get desired joint positions (used only in joint position control)",
	"set control point i.e. end-effector in root frame or desired contact point in link frame (cm)",
    "get control point i.e. end-effector in root frame or desired contact point in link frame (cm)",
    "get current cartesian pose of control point i.e. control point in root frame (cm)",
    "set link of control point",
    "get link of control point",
    "set desired target position",
    "get desired target position",
    "set desired joint torques (used only in torque control)",
    "set desired torque of one joint (params: jnt index and desired torque)",
    "get desired joint torques (used only in torque control)",
    "get current joint torques",
	"set desired wrench of end effector (used in force and paral ctrl)",
    "get desired wrench of end effector (used in force and paral ctrl)",
    "get current wrench applied by contact point on environment",
    "set cut frequency of low pass filter applied to contact forces (in Hz)",
    "get cut frequency of low pass filter applied to contact forces (in Hz)",
    "set cut frequency of low pass filter applied to fd (in Hz)",
    "get cut frequency of low pass filter applied to fd (in Hz)",
    "set cut frequency of low pass filter applied to taod (in Hz)",
    "get cut frequency of low pass filter applied to taod (in Hz)",
    "set trajectory time in sec",
    "get trajectory time in sec",
    "activate simulation mode (computed torques are not commanded to motors)",
    "deactivate simulation mode (computed torques are not commanded to motors)",
    "block specified joint(s) in current position",
    "unblock specified joint(s)",
    "get 0 if specified joint is blocked, 1 otherwise, if no joint is specified it returns all",
    "set the pseudo-inverse damping factor",
    "get the pseudo-inverse damping factor",
    "set the gain for low level control compensation",
    "get the gain for low level control compensation"
};

class IController
{
protected:
    std::vector<std::string>    commandList;        // list of commands accepted by the planner
    std::vector<std::string>    commandDesc;        // description of commands accepted by the planner

    std::string             name;               // controller name
    VerbosityLevel          verbose;
    int                     period;             // controller period
    DSemaphore              mutex;              // semaphore managing the access to all controller variables
    Status                  thread_status;      // the status of the controller (OK, ERROR)
                                                // as soon as the status is ERROR, the module calls stop() on this thread
        
    ControlLaw          ctrlLaw;        // the type of control used
    CommandMode         cmdMode;        // SIMULATION: no torques are sent to the motors
    // ROBOT VARIABLES
    int                 dof;            // degrees of freedom
    RobotStatus         robot;          // feedback of the controller (robot current status)
    CtrlRef             ctrlRef;        // reference of the controller (desired cart force, cart position, joint torques, joint pos)
    // end effector force = force exherted by the e.e. upon the environment
    Matrix              H_r_c;          // rototranslation from root to pos ctrl point link
    Matrix              R_r_c;          // rotation from root to pos ctrl point link
    Matrix              H_r_c2;         // rototranslation from root to contact point link
    Matrix              R_r_c2;         // rotation from root to contact point link
    skinContactList     partContList;   // list of contacts on interested body part
    skinContact*        contact;        // contact chosen for the controller
    Vector              fd;             // desired contact force/torque
    Vector              qd;             // desired joint position
    
    Vector              fSmooth;        // measured contact wrench after smooth filtering
    Vector              x;              // position of pos ctrl point (root ref frame)
    Vector              x2;             // position of force ctrl point (root ref frame)
    Vector              dx;             // velocity of ctrl point (root ref frame)
    Vector              ddx;            // acceleration of ctrl point (root ref frame)
    Matrix              J;              // jacobian of ctrl point
    Matrix              dJ;             // time derivative of jacobian of ctrl point

    // FILTERING
    FirstOrderLowPassFilter*    lpFilter_f;         // measured force/torque low pass filter
    FirstOrderLowPassFilter*    lpFilter_fSmooth;   // measured force/torque low pass filter (with lower cut frequency)
    FirstOrderLowPassFilter*    lpFilter_CoP;       // contact point low pass filter
    FirstOrderLowPassFilter*    lpFilter_ref;       // commanded reference low pass filter
    FirstOrderLowPassFilter*    lpFilter_fd;        // desired force/torque low pass filter
    minJerkRefGen*              trajGen_xd;         // desired position min-jerk filter
    minJerkTrajGen*             trajGen_ctrlPnt;    // position control point min-jerk filter
    minJerkRefGen*              trajGen_qd;         // desired joint position min-jerk filter
    AWLinEstimator*             dqEst;              // derivative estimator for joint velocities
    AWQuadEstimator*            ddqEst;             // double derivative estimator for joint accelerations

    bool                controlForceNorm;   // true iff the desired force direction has to be the same as the measured force direction

    // CONTROLLERS
    map<ControlLaw, IControlLaw*> ctrlLaws;     // available control laws
    double              feasibility;            // value assessing the feasibility of the current task (0 unfeasible, 1 perfectly feasible)
    
    bool                ctrlLawChanged;     // true iff the ctrl mode has just been changed
    BodyPart            bodyPart;			// controlled body part (LEFT_ARM or RIGHT_ARM)

    RobotArm*           robotChain;         // kinematics/dynamics representation of robot chain
    RobotInterfaces*    robotInt;           // interfaces to communicate with the robot
    
	Vector              commandRef;         // commandRef commanded to the motors  
    Vector              k_comp;             // gain for low level control compensation
    
public:
    enum IControllerCommand{
        stop_ctrl,      float_ctrl,     jpos_ctrl,      pos_ctrl,       torque_ctrl,    
        force_ctrl,     paral_ctrl,     dparal_ctrl,    cont_ctrl,      press_ctrl,     
        reach_ctrl,     get_ctrl,
        set_qd,         get_qd,
        set_ctrlPnt,    get_ctrlPnt,    get_x,
        set_ctrlLink,   get_ctrlLink,
        set_xd,         get_xd,
        set_taud,       set_taud_j,     get_taud,       get_tau,
        set_fd,         get_fd,         get_f,
        set_alphaF,     get_alphaF,     set_alphaFd,    get_alphaFd,    
        set_alphaTaod,  get_alphaTaod,  set_trajTime,   get_trajTime,
        sim_on,         sim_off,        block_joint,    unblock_joint,  is_blocked,
        set_damp,       get_damp,       set_kcomp,      get_kcomp,      IControllerCommandSize};

    IController() throw();
    IController(std::string _name, std::string robotname, int _period, BodyPart _bodyPart, VerbosityLevel _verbose=NO_VERBOSE) throw();
	
    // *******************************************************************************************************
    // *                                              SET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    void safeSet(T& var, const T& value){
        return iCub::skinForceControl::safeSet(var, value, mutex);
    }
    inline Bottle setQd(Bottle param) throw(){
        Vector v;
        Status s = bottleToVector(param, v);
        if(!s)  return Bottle(s.toString());
        s = setQd(v);
        return s?Bottle():Bottle(s.toString());
    }
    inline Status setQd(Vector _qd) throw(){
        if(_qd.size() != dof)
            return Status("Qd size is not equal to ctrl dof ("+toString(_qd.size())+", "+toString(dof)+ ")");
        Vector qMax = robotChain->getJointBoundMax() * CTRL_RAD2DEG;
        Vector qMin = robotChain->getJointBoundMin() * CTRL_RAD2DEG;
        for(int i=0;i<dof;i++)
            if( _qd(i)>qMax(i) )
                _qd(i) = qMax(i);
            else if( _qd(i)<qMin(i) )
                _qd(i) = qMin(i);
        //return Status("Qd "+toString(i)+" is not inside the joint limits "+toString(qMin(i))+" < "+toString(_qd(i))+" < "+toString(qMax(i))+" )");
        mutex.wait();
        {
            qd = _qd;
        }
        mutex.post();
        return Status();
    }
    inline Status setCtrlPnt(Vector p) throw(){
        Status s;
        mutex.wait();
        {
            if(p.size() != ctrlRef.ctrlPointRef.size() && p.size()!=3)
                s.addErrMsg("Ctrl point wrong size "+toString(p.size()));
            else
            {
                ctrlRef.ctrlPointRef = p.subVector(0,2);
                if(ctrlLaws[ctrlLaw] != NULL)
                    ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
            }
        }
        mutex.post();
        return s;
    }
    inline Status setCtrlPnt(unsigned int _linkNum, Vector p) throw(){
        Status s;
        mutex.wait();
        {
            if(p.size() != ctrlRef.ctrlPointRef.size() && p.size()!=3)
                s.addErrMsg("Control point wrong size "+toString(p.size()));
            else{ 
                ctrlRef.linkNumber = _linkNum;
                ctrlRef.ctrlPointRef = p.subVector(0,2);
                if(ctrlLaws[ctrlLaw] != NULL)
                    ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
            }
        }
        mutex.post();
        return s;
    }
    inline Status setCtrlPntLink(unsigned int _linkNum) throw(){
        Status s;
        mutex.wait();
        {
            ctrlRef.linkNumber = _linkNum;
            if(ctrlLaws[ctrlLaw] != NULL)
                ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
        }
        mutex.post();
        return s;
    }
    inline Status setXd(Vector _xd) throw(){
        Status s;
        mutex.wait();
        {
            if(_xd.size() != 3 && _xd.size()!=4)
                s.addErrMsg("Xd wrong size "+toString(_xd.size()));
            else
            {
                if(_xd.size()==3)
                    ctrlRef.xRef = _xd;
                else if(_xd.size()==4){
                    ctrlRef.xRef = (robotChain->getH(int(_xd[0]))*cat(_xd.subVector(1,3),1.0)).subVector(0,2);
                }
            }
        }
        mutex.post();
        return s;
    }
    inline Status setTaud(Vector _taud) throw(){
        Status s;
        mutex.wait();
        {
            if(_taud.size() != ctrlRef.torques.size())
                s.addErrMsg("Taud wrong size "+toString(_taud.size()));
            else
            {
                ctrlRef.torques = _taud;
                if(ctrlLaws[ctrlLaw] != NULL)
                    ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
            }
        }
        mutex.post();
        return s;
    }
    inline Status setTaud(unsigned int jnt, double _taud) throw(){
        Status s;
        mutex.wait();
        {
            if(jnt>=ctrlRef.torques.size())
                s.addErrMsg("Joint index out of range "+toString(jnt));
            else
            {
                ctrlRef.torques[jnt] = _taud;
                if(ctrlLaws[ctrlLaw] != NULL)
                    ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
            }
        }
        mutex.post();
        return s;
    }
    inline void setTaud(double _taud) throw(){
        mutex.wait();
        {
            ctrlRef.torques = _taud;
            if(ctrlLaws[ctrlLaw] != NULL)
                ctrlLaws[ctrlLaw]->referenceChanged(ctrlRef, robot);
        }
        mutex.post();
    }
    inline Status setFd(Vector _fd) throw(){
        Status s;
        mutex.wait();
        {
            if(_fd.size() != fd.size() && _fd.size()!=3)
                s.addErrMsg("Fd wrong size "+toString(_fd.size()));
            else{
                for(unsigned int i=0;i<_fd.size();i++) 
                    fd[i] = _fd[i];
            }
        }
        mutex.post();
        return s;
    }
    inline Status setKcomp(float v) throw() {
        if(v<0.0)
            return Status("error: kcomp cannot be set to negative values: "+toString(v));
        safeSet(k_comp, Vector(dof, v));
        return Status();
    }
    inline Status setKcomp(Vector v) throw() {
        for(unsigned int i=0;i<v.size(); i++)
            if(v(i)<0.0)
                return Status("error: kcomp cannot be set to negative values: "+toString(v(i)));
        safeSet(k_comp, v);
        return Status();
    }
    inline Status setAlphaF(float _alpha) throw(){
        if(_alpha<=0.0)
            return Status("Cut frequency is negative: "+toString(_alpha));
        mutex.wait();
        lpFilter_f->setCutFrequency(_alpha);
        mutex.post();
        return Status();
    }
    inline Status setAlphaFd(float _alpha) throw(){
        if(_alpha<=0.0)
            return Status("AlphaFd negative: "+toString(_alpha));
        mutex.wait();
        lpFilter_fd->setCutFrequency(_alpha);
        mutex.post();
        return Status();
    }
    inline Status setAlphaTaod(float _alpha) throw(){
        if(_alpha<=0.0)
            return Status("AlphaTaod negative: "+toString(_alpha));
        mutex.wait();
        lpFilter_ref->setCutFrequency(_alpha);
        mutex.post();
        return Status();
    }
    inline Status setTrajTime(float _trajTime) throw(){
        if(_trajTime<0.1)
            return Status("Trajectory time cannot be less than 0.1 sec");
        mutex.wait();
        trajGen_xd->setT(_trajTime);
        trajGen_qd->setT(_trajTime);
        mutex.post();
        return Status();
    }
    inline Status setPinvDamp(float _damp) throw(){
        if(_damp<0.0)
            return Status("Damping factor cannot be negative");
        mutex.wait();
        ctrlRef.pinvDamp = _damp;
        mutex.post();
        return Status();
    }
    inline Status blockJoint(int j) throw(){
        if(j<0 || j>=dof)
            return Status("Joint index out of range: "+toString(j));
        if(robot.activeJoints[j]==0.0)
            return Status("Joint "+toString(j)+" was already blocked");
        mutex.wait();
        {
            robot.activeJoints[j] = 0.0;
            if(ctrlLaw!=NO_CONTROL && cmdMode==REAL)
                robotChain->setPositionMode(j);
        }
        mutex.post();
        return Status();
    }
    inline Status unblockJoint(int j) throw(){
        if(j<0 || j>=dof)
            return Status("Joint index out of range: "+toString(j));
        // no need to synchronize
        if(robot.activeJoints[j]==1.0)
            return Status("Joint "+toString(j)+" was already active");
        mutex.wait();
        {
            robot.activeJoints[j] = 1.0;
            if(ctrlLaw!=NO_CONTROL && cmdMode==REAL)
                robotChain->setCtrlMode(ctrlLaws[ctrlLaw]->getControlMode(), j);
        }
        mutex.post();
        return Status();
    }
    inline Status isJointBlocked(int j, bool& isBlocked) throw(){
    	if(j<0 || j>=dof)
            return Status("Joint index out of range: "+toString(j));
        safeSet(isBlocked, robot.activeJoints[j]==0.0);
        return Status();
    }
    inline Status isJointBlocked(Vector &isBlocked) throw(){
        safeSet(isBlocked, robot.activeJoints);
        return Status();
    }
    inline void setCtrlLaw(const ControlLaw &cm){
        mutex.wait();
        if(ctrlLaw != cm){
            ctrlLawChanged = true;
            ctrlLaw = cm;
        }
        mutex.post();
    }
    inline void setSimMode(bool on){
        safeSet(cmdMode, on?SIMULATION:REAL);
    }

    // *******************************************************************************************************
    // *                                              GET METHODS                                            *
    // *******************************************************************************************************
    template<class T>
    T safeGet(const T& value){
        return iCub::skinForceControl::safeGet(value, mutex);
    }
    inline ControlLaw getCtrlLaw(){ return safeGet(ctrlLaw); }
	inline Status getStatus(){ return safeGet(thread_status); }
    inline Status getPidN(unsigned int &pidN, ControlLaw cm=NO_CONTROL){
        Status s;
        mutex.wait();
        {
            if(cm==NO_CONTROL)
                cm = ctrlLaw;
            if(ctrlLaws[cm] == NULL)
                s.addErrMsg("Error while trying to access to the ctrl law " + ControlLaw_desc[cm]);
            else
                pidN = ctrlLaws[cm]->getPidN(); 
        }
        mutex.post();
        return s;
    }
    inline unsigned int getPidN(ControlLaw cm=NO_CONTROL){
        unsigned int res;
        mutex.wait();
        {
            if(cm==NO_CONTROL)
                cm = ctrlLaw;
            if(ctrlLaws[cm] == NULL)
                res = 0;
            else
                res = ctrlLaws[cm]->getPidN();
        }
        mutex.post();
        return res;
    }
    inline unsigned int getN(ControlLaw cm=NO_CONTROL){
        unsigned int res;
        mutex.wait();
        {
            if(cm==NO_CONTROL)
                cm = getCtrlLaw();
            if(ctrlLaws[cm] == NULL)
                res = 0;
            res = ctrlLaws[cm]->getN();
        }
        mutex.post();
        return res;
    }
    inline Vector getQd(){ return safeGet(qd); }
    inline Vector getXd(){ return safeGet(ctrlRef.xRef); }
    inline Vector getX(){ return safeGet(x); }
    inline Vector getTau(){ return safeGet(robot.torques); }
    inline Vector getTaud(){ return safeGet(ctrlRef.torques); }
    inline Vector getF(){ return safeGet(robot.wrench); }
    inline Vector getFd(){ return safeGet(fd); }
    inline Vector getKcomp(){ return safeGet(k_comp); }
    inline float getAlphaF(){ 
        mutex.wait();
        double res = lpFilter_f->getCutFrequency();
        mutex.post();
        return (float)res;
    }
    inline float getAlphaFd(){
        mutex.wait();
        double res = lpFilter_fd->getCutFrequency();
        mutex.post();
        return (float)res;
    }
    inline float getAlphaTaod(){ 
        mutex.wait();
        double res = lpFilter_ref->getCutFrequency();
        mutex.post();
        return (float)res;
    }
    inline float getTrajTime(){ 
        mutex.wait();
        double res = trajGen_xd->getT();
        mutex.post();
        return (float)res;
    }
    inline CtrlRef getCtrlRef(){ return safeGet(ctrlRef); }
    inline RobotStatus getRobotStatus() { return safeGet(robot); }
    inline double getFeasibility() { return safeGet(feasibility); }
    inline Vector getCtrlPoint() { return cat(safeGet(ctrlRef.linkNumber), safeGet(ctrlRef.ctrlPoint)); }
    inline unsigned int getCtrlPointLink() { return safeGet(ctrlRef.linkNumber); }

    inline vector<string> getCommandList() {
        vector<string> cl = commandList;
        mutex.wait();
        if(ctrlLaws[ctrlLaw] != NULL){
            vector<string> cla = ctrlLaws[ctrlLaw]->getCommandList();
            cl.insert(cl.end(), cla.begin(), cla.end());
        }
        mutex.post();
        return cl; 
    }

    inline vector<string> getCommandDesc() {
        vector<string> cd = commandDesc;
        mutex.wait();
        if(ctrlLaws[ctrlLaw] != NULL){
            vector<string> cda = ctrlLaws[ctrlLaw]->getCommandDesc();
            cd.insert(cd.end(), cda.begin(), cda.end());
        }
        mutex.post();
        return cd;
    }

    virtual Status respond(const Bottle& command, Bottle& reply) 
    {
	    unsigned int cmdId;
        Bottle params;
	    if(!identifyCommand(command, commandList, cmdId, params)){
            // if the command is not recognized, send it to the lower level
            // not sure if the synchronization is needed here
            if(ctrlLaws[getCtrlLaw()]!=NULL) 
                return ctrlLaws[ctrlLaw]->respond(command, reply);
		    return Status("Command unknown");
	    }
        return respond(cmdId, params, reply);
    }

    virtual Status respond(unsigned int cmdId, const Bottle& param, Bottle& reply)
    {
        Status s;
	    switch( cmdId ){
            case stop_ctrl:     setCtrlLaw(NO_CONTROL);     break;
            case float_ctrl:    setCtrlLaw(FLOAT_CTRL);     break;
            case jpos_ctrl:     setCtrlLaw(JPOS_CTRL);      break;
            case pos_ctrl:      setCtrlLaw(POS_CTRL);       break;
            case torque_ctrl:   setCtrlLaw(TORQUE_CTRL);    break;
            case force_ctrl:    setCtrlLaw(FORCE_CTRL);     break;
            case paral_ctrl:    setCtrlLaw(PARAL_CTRL);     break;
            case dparal_ctrl:   setCtrlLaw(DYN_PARAL_CTRL); break;
            case cont_ctrl:     setCtrlLaw(CONTACT_CTRL);   break;
            case press_ctrl:    setCtrlLaw(PRESS_CTRL);     break;
            case reach_ctrl:    setCtrlLaw(SAFE_REACH_RIGID_CTRL);     break;
            case get_ctrl:      reply.addString(ControlLaw_desc[ctrlLaw].c_str());  break;

            case get_qd:        addToBottle(reply, getQd());        break;
            case get_ctrlPnt:   addToBottle(reply, getCtrlPoint()); break;
            case get_xd:        addToBottle(reply, getXd());        break;
            case get_taud:      addToBottle(reply, getTaud());      break;
            case get_tau:       addToBottle(reply, getTau());       break;
            case get_x:         addToBottle(reply, getX());         break;
            case get_fd:        addToBottle(reply, getFd());        break;
            case get_f:         addToBottle(reply, getF());         break;
            case get_alphaF:    reply.addDouble(getAlphaF());       break;
            case get_alphaFd:   reply.addDouble(getAlphaFd());      break;
            case get_alphaTaod: reply.addDouble(getAlphaTaod());    break;
            case get_trajTime:  reply.addDouble(getTrajTime());     break;
            case get_ctrlLink:  reply.addInt(getCtrlPointLink());   break;
            case get_damp:      reply.addDouble(ctrlRef.pinvDamp);  break;
            case get_kcomp:     addToBottle(reply, getKcomp());     break;
            case sim_on:        setSimMode(true);  break;
            case sim_off:       setSimMode(false); break;

            case set_qd:    reply = setQd(param);      break;
            case set_ctrlPnt:
                {
                    Vector p;
                    if(param.size()==4){
                        if(!param.get(0).isInt())
                            s.addErrMsg("Error: first parameter (link number) is not an int");
                        s = s ? bottleToVector(param.tail(), p) : s;
                        s = s ? setCtrlPnt((unsigned int)param.get(0).asInt(), p) : s;
                    }
                    else{
                        s = bottleToVector(param, p);
                        s = s ? setCtrlPnt(p) : s;
                    }
                    break;
                }
            case set_ctrlLink:
                {
                    if(param.size()<1 || !param.get(0).isInt() || param.get(0).asInt()<0)
                        s.addErrMsg("Error: wrong input parameters, should be one positive int");
                    else
                        s = setCtrlPntLink(param.get(0).asInt());
                    break;
                }
            case set_xd:
                {
                    Vector xd;
                    s = bottleToVector(param, xd);
                    s = s ? setXd(xd) : s;
                    break;
                }
            case set_taud:
                {
                    if(param.size()>1){
                        Vector taud;
                        s = bottleToVector(param, taud);
                        if(s)
                            s = setTaud(taud);
                    }else
                        setTaud(param.get(0).asDouble());
                    break;
                }
            case set_taud_j:
                {
                    if(param.size()!=2 || !param.get(0).isInt() || !param.get(1).isDouble())
                        s.addErrMsg("Error: wrong input parameters, should be (int double)");
                    else
                        s = setTaud(param.get(0).asInt(), param.get(1).asDouble());
                    break;
                }
            case set_fd:
                {
                    Vector fd;
                    s = bottleToVector(param, fd);
                    if(s)
                        s = setFd(fd);
                    break;
                }
            case set_alphaF:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to alpha is missing");
                    else
                        s = setAlphaF((float)param.get(0).asDouble());
                    break;
                }
            case set_alphaFd:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to alphaFd is missing");
                    else
                        s = setAlphaFd((float)param.get(0).asDouble());
                    break;
                }
            case set_alphaTaod:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to alphaTaod is missing");
                    else
                        s = setAlphaTaod((float)param.get(0).asDouble());
                    break;
                }
            case set_trajTime:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to trajectory time is missing");
                    else
                        s = setTrajTime((float)param.get(0).asDouble());
                    break;
                }
            case set_damp:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to damping factor is missing");
                    else
                        s = setPinvDamp((float)param.get(0).asDouble());
                    break;
                }
            case set_kcomp:
                {
                    if(param.size()==0 || !param.get(0).isDouble())
                        s.addErrMsg("Error: a float value to assign to kcomp is missing");
                    else if(param.size()==1)
                        s = setKcomp((float)param.get(0).asDouble());
                    else{
                        Vector v;
                        bottleToVector(param, v);
                        s = setKcomp(v);
                    }
                    break;
                }
            case block_joint:
                {
                    for(int i=0; i<param.size(); i++){
                        s = blockJoint(param.get(i).asInt());
                        if(!s)
                            break;
                    }
                    break;
                }
            case unblock_joint:
                {
                    for(int i=0; i<param.size(); i++){
                        s = unblockJoint(param.get(i).asInt());
                        if(!s)
                            break;
                    }
                    break;
                }
            case is_blocked:
                {
                    if(param.size()==0){
                        Vector r;
                        isJointBlocked(r);
                        reply.addString(r.toString(0).c_str());
                        break;
                    }
                    bool b;
                    for(int i=0; i<param.size(); i++){
                        s = isJointBlocked(param.get(i).asInt(), b);
                        if(!s)
                            break;
                        reply.addString(b?"yes ":"no ");
                    }
                    break;
                }

		    default:
                return Status("ERROR: This command is known but it is not managed in the code.");
	    }
        if(s && reply.toString()=="")
            reply.addString( (commandList[cmdId]+" command received.").c_str());
        else if(!s)
            reply.addString(s.toString());
	    return Status();
    }
};


const std::string IControlThreadCommand_s[]  = {"firmware white",  "firmware black",   "firmware old"};

// the order in IControlThreadCommand_desc must correspond to the order in IControlThreadCommand_s
const std::string IControlThreadCommand_desc[]  = {
    "set new gains for white iCub in firmware torque pid",
    "set new gains for black iCub in firmware torque pid",
    "set old gains in firmware torque pid" };

class controlThread: public yarp::os::RateThread, public IController
{
private:
    std::string             wholeBodyName;      // wholeBodyDynamics module name
    
    // NETWORK DELAY MANAGEMENT
    double                  extContTimestamp;   // timestamp of the last reading of port_ext_contacts
    double                  encTimestamp;       // timestamp of the last reading of the encoders
    double                  torqueTimestamp;    // timestamp of the last reading of the joint torques
    double                  timePre;
    vector<double>          durations;
    vector<double>          timestamps;
    vector<string>          durationDesc;

    // PORTS
    BufferedPort<skinContactList>   *port_ext_contacts; // input port reading external contacts
    BufferedPort<Vector>            *port_ext_ft_sens;  // input port reading external wrench seen at the F/T sensor
	BufferedPort<Vector>            *port_torques;      // output port sending the same torques that are commanded to the motors
    BufferedPort<Vector>            *port_monitor;      // output port sending data for monitoring the controller
    RpcClient                       torqueCtrlTestPort;

    SfcMonitorData data;                                // vector sent on the monitor port
    skinContact contactToSend;                          // contact sent on the monitor port

#ifdef PREDICT_ERROR
    Vector e_w, e_p, e_wd, e_tau;
#endif
	
    void sendMsg(const string &msg, const MsgType &type=MSG_INFO) throw();
    void sanityCheck() throw();
    void updateRobotStatus() throw();
    void initCtrlLaw(const ControlLaw &cm) throw();
    double computeCtrlRef(const ControlLaw &cm) throw();
    void sendCtrlRef(const ControlLaw &cm) throw();
    void sendMonitorData(const ControlLaw &cm) throw();

public:	
    enum IControlThreadCommand{ firmware_white=IControllerCommandSize,  firmware_black,  firmware_old, IControlThreadCommandSize};

    controlThread(std::string _moduleName, string robotName, int _period, string _wholeBodyName, 
                    BodyPart _bodyPart, VerbosityLevel _verbose=NO_VERBOSE) throw();
	
    bool threadInit();	
    void run();
    void threadRelease();

    // *******************************************************************************************************
    // *                                              SET METHODS                                            *
    // *******************************************************************************************************
    Status setOldFirmwarePidGains() throw(){
        return setFirmwarePidGains(robotInt, bodyPart, FIRMWARE_KP_OLD, FIRMWARE_KI_OLD, FIRMWARE_KD_OLD, 
            FIRMWARE_SHIFT_OLD, FIRMWARE_MAX_OLD, FIRMWARE_FC_OLD);
    }

    Status setNewFirmwarePidGainsBlack() throw(){
        return setFirmwarePidGains(robotInt, bodyPart, FIRMWARE_KP_NEW_BLACK, FIRMWARE_KI_NEW_BLACK, FIRMWARE_KD_NEW_BLACK, 
            FIRMWARE_SHIFT_NEW_BLACK, FIRMWARE_MAX_NEW_BLACK, FIRMWARE_FC_NEW_BLACK);
    }

    Status setNewFirmwarePidGainsWhite() throw(){
        return setFirmwarePidGains(robotInt, bodyPart, FIRMWARE_KP_NEW_WHITE, FIRMWARE_KI_NEW_WHITE, FIRMWARE_KD_NEW_WHITE, 
            FIRMWARE_SHIFT_NEW_WHITE, FIRMWARE_MAX_NEW_WHITE, FIRMWARE_FC_NEW_WHITE);
    }

    virtual Status respond(const Bottle& command, Bottle& reply){
	    return IController::respond(command, reply);
    }

    virtual Status respond(unsigned int cmdId, const Bottle& param, Bottle& reply)
    {
        Status s;
	    switch( cmdId ){
            case firmware_black:    s=setNewFirmwarePidGainsBlack();  break;
            case firmware_white:    s=setNewFirmwarePidGainsWhite();  break;
            case firmware_old:      s=setOldFirmwarePidGains();       break;
		    default:
                return IController::respond(cmdId, param, reply);
	    }
        if(s && reply.toString()=="")
            reply.addString( (commandList[cmdId]+" command executed.").c_str());
        else if(!s)
            reply.addString(s.toString());
	    return Status();
    }

};


}

} // end namespace

#endif
