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

#include <iCub/skinForceControl/controlLaws.h>
#include <iCub/skinForceControl/skinForceControlLib.h>
#include <iCub/skinForceControl/util.h>
#include <iCub/skinForceControl/utilRobotics.h>
#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <sstream>

using namespace std;
using namespace yarp;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;

namespace iCub
{

namespace skinForceControl
{

//====================================
//
//		IControlLaw
//
//====================================
void IControlLaw::construct(iDynLimb* _limb){
    name = "IControlLaw";
    shortName = "ictrl";
    mutex.setName(name+"Mutex");
    limb = _limb->asChain();
    N = _limb ? limb->getN() : 0;
    pidN = 0;
    pid = 0;
    commandList.assign(IControlLawCommand_s, IControlLawCommand_s+IControlLawCommandSize);
    commandDesc.assign(IControlLawCommand_desc, IControlLawCommand_desc+IControlLawCommandSize);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void IControlLaw::construct(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd){
    construct(_limb);
    
    period = _period;
    pidN = _kp.size();
    if(_ki.size()!=pidN || _kd.size()!=pidN){
        globalStatus = Status(false, "Sizes of kp, kd and ki are not equal");
        return;
    }
    Vector N(pidN),  Tt(pidN), satDown(pidN), satUp(pidN);
    Matrix satLim(pidN,2);
    N=DEFAULT_N;
    Tt=DEFAULT_TT;
    satDown=DEFAULT_TORQUE_SAT_LIM_DOWN;
    satUp=DEFAULT_TORQUE_SAT_LIM_UP;
    satLim.setCol(0, satDown);
    satLim.setCol(1, satUp);
    initPid(_period,_kp,_ki,_kd,N,Tt,satLim);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::compute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    if(globalStatus==false)
        return globalStatus;
    // acquire the mutex to be sure that the PID parameters are not modified while the 
    // torque computation is ongoing
    mutex.wait();
    Status res = abstractCompute(ref, s, out, feasibility);
    mutex.post();
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::setParameter(const string &key, const Vector &value){
    if(key=="Kp" || key=="Ki" || key=="Kd"){
        if(pid==NULL)
            return Status("Error while setting param "+key+". This ctrl law does not have a PID.");
        if(value.size() != pidN)
            return Status("Error while setting param "+key+", expected size: "+toString(pidN)+"; found size: "+toString(value.size()));
        Bottle b;
        helperPID::addVectorToOption(b, key.c_str(), value);
        mutex.wait();
        pid->setOptions(b);
        mutex.post();
        return Status();
    }
    return Status(key+" not found");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::getParameter(const string &key, Vector& v) {
    if(key=="Kp" || key=="Ki" || key=="Kd"){
        if(pid==NULL)
            return Status("Error while getting param "+key+". This ctrl law does not have a PID.");
        Bottle b;
        mutex.wait();
        pid->getOptions(b);
        mutex.post();
        int size=pidN;
        v.resize(size);
        helperPID::getVectorFromOption(b, key.c_str(), v, size);
        return Status();
    }
    return Status(key+" not found");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void IControlLaw::initPid(int period, const Vector &kp, const Vector &ki, const Vector &kd, 
                        const Vector &N, const Vector &Tt, const Matrix &satLim){
    Vector W(pidN);
    W=1.0;
    pid = new parallelPID(period/1000.0,kp,ki,kd,W,W,W,N,Tt,satLim);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::reset(const CtrlRef &ref, const RobotStatus &s){
    if(pid==NULL)
        return Status("Error while trying to reset the control law. This ctrl law does not have a PID.");
    mutex.wait();
    pid->reset(zeros(pidN));
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::respond(const Bottle& command, Bottle& reply){
    Bottle params;
    unsigned int cmdId;
    if( ! identifyCommand(command, commandList, cmdId, params))
        return Status("Unknown command");
    return respond(cmdId, params, reply);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kp:    reply.addString(getKp().toString(3).c_str()); break;
        case get_ki:    reply.addString(getKi().toString(3).c_str()); break;
        case get_kd:    reply.addString(getKd().toString(3).c_str()); break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s) break;
                switch(cmdId){
                    case set_kp:    
                        if(v.size()==1)
                            s = setKp(Vector(pidN,v[0]));  
                        else
                            s = setKp(v);
                        break;
                    case set_ki:    
                        if(v.size()==1)
                            s = setKi(Vector(pidN,v[0]));  
                        else
                            s = setKi(v);
                        break;
                    case set_kd:    
                        if(v.size()==1)
                            s = setKd(Vector(pidN,v[0]));
                        else
                            s = setKd(v);
                        break;
                    // return false if the command is not recognized
                    default:        return Status("Command not managed");
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}


//====================================
//
//		JointBoundControlLaw
//
//====================================
JointBoundControlLaw::JointBoundControlLaw(){
    init(zeros(N), zeros(N), zeros(N));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
JointBoundControlLaw::JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd):
        IControlLaw(_limb, _period, _kp, _ki, _kd){
    init(zeros(N), zeros(N), zeros(N));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
JointBoundControlLaw::JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds):
        IControlLaw(_limb, _period, _kp, _ki, _kd){
    init(_kj, _activationThresholds, _deactivationThresholds);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
JointBoundControlLaw::JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds)
        :IControlLaw(_limb){
    init(_kj, _activationThresholds, _deactivationThresholds);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void JointBoundControlLaw::init(const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds){
    int cmdSize =JointBoundControlLawCommandSize - commandList.size();
    commandList.insert(commandList.end(), JointBoundControlLawCommand_s, JointBoundControlLawCommand_s+cmdSize);
    commandDesc.insert(commandDesc.end(), JointBoundControlLawCommand_desc, JointBoundControlLawCommand_desc+cmdSize);

    name = "JointBoundControlLaw";
    shortName = "jbcl";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;

    jointOverBound.resize(N,0);
    globalStatus = globalStatus && setKj(_kj);
    activationThresholds = _activationThresholds;
    globalStatus = globalStatus && setDeactivationThresholds(_deactivationThresholds);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void JointBoundControlLaw::updateInternalThresholds(){
    qOnMin = limb->getJointBoundMin()*CTRL_RAD2DEG + activationThresholds;
    qOnMax = limb->getJointBoundMax()*CTRL_RAD2DEG - activationThresholds;
    qOffMin = limb->getJointBoundMin()*CTRL_RAD2DEG + deactivationThresholds;
    qOffMax = limb->getJointBoundMax()*CTRL_RAD2DEG - deactivationThresholds;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JointBoundControlLaw::computeJntBoundCtrl(const RobotStatus &s, Vector &out){
    Vector qDir(N,0.0);
    for(unsigned int i=0;i<N;i++){
        if(s.activeJoints[i]==1.0){
            if(jointOverBound[i]==1){
                if(s.q[i]>qOffMin[i] && s.q[i]<qOffMax[i]){
                    //printf("Joint %d is in safe position now: %.1f<%.1f<%.1f\n", i, qOffMin[i], s.q[i], qOffMax[i]);
                    jointOverBound[i] = 0;
                }
                else
                    qDir[i] = (s.q[i]<=qOffMin[i]) ? 1 : -1;  // direction the joint has to move into to go away from the close bound
            }
            else if(s.q[i]<qOnMin[i] || s.q[i]>qOnMax[i]){
                //printf("Joint %d too close to bound: %.1f<%.1f<%.1f\n", i, qOnMin[i], s.q[i], qOnMax[i]);
                jointOverBound[i] = 1;
                qDir[i] = (s.q[i]<qOnMin[i]) ? 1 : -1;  // direction the joint has to move into to go away from the close bound
            }
        }
    }
    out = jointOverBound * kj * qDir;
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void JointBoundControlLaw:: mergeJntBoundCtrl(Vector &jbTorque, Vector &ctrlTorque){
    for(unsigned int i=0;i<N;i++)
        if(jointOverBound[i]==1 && ctrlTorque[i]*jbTorque[i]<0)
            ctrlTorque[i] = jbTorque[i];
        else
            ctrlTorque[i] += jbTorque[i];
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JointBoundControlLaw::setKj(const Vector &_kj) {
    if(_kj.size() != N)
        return Status("Wrong dimension of kj ("+toString(_kj.size())+"!="+toString(N)+")");
    mutex.wait();
    kj=_kj;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JointBoundControlLaw::setActivationThresholds(const yarp::sig::Vector &_activationThresholds){
    for(unsigned int i=0;i<N;i++)
        if(_activationThresholds[i] > deactivationThresholds[i])
            return Status("Joint "+toString(i)+" bound activation threshold greater than deactivation threshold ("+
                toString(_activationThresholds[i])+">"+toString(deactivationThresholds[i])+")");
    mutex.wait();
    activationThresholds = _activationThresholds;
    updateInternalThresholds();
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JointBoundControlLaw::setDeactivationThresholds(const yarp::sig::Vector &_deactivationThresholds){
    for(unsigned int i=0;i<N;i++)
        if(activationThresholds[i] > _deactivationThresholds[i])
            return Status("Joint "+toString(i)+" bound deactivation threshold less than activation threshold ("+
                toString(activationThresholds[i])+">"+toString(_deactivationThresholds[i])+")");
    mutex.wait();
    deactivationThresholds = _deactivationThresholds;
    updateInternalThresholds();
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JointBoundControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kj:    reply.addString(getKj().toString(3).c_str()); break;
        case get_act:   reply.addString(getActivationThresholds().toString(3).c_str()); break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s) break;
                switch(cmdId){
                    case set_kj:    
                        if(v.size()==1)
                            s = setKj(Vector(N, v[0]));
                        else
                            s = setKj(v);
                        break;
                    case set_act:   
                        if(v.size()==1)
                            s = setActivationThresholds(Vector(N, v[0]));  
                        else
                            s = setActivationThresholds(v);
                        break;
                    default:        
                        return IControlLaw::respond(cmdId, params, reply);
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}



//====================================
//
//		ForceFieldJointBoundControlLaw
//
//====================================
ForceFieldJointBoundControlLaw::ForceFieldJointBoundControlLaw(){ init(); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ForceFieldJointBoundControlLaw::ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd)
        :JointBoundControlLaw(_limb, _period, _kp, _ki, _kd){ init(); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ForceFieldJointBoundControlLaw::ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds)
        :JointBoundControlLaw(_limb, _period, _kp, _ki, _kd, _kj, _activationThresholds, _deactivationThresholds){ init(); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ForceFieldJointBoundControlLaw::ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds)
        :JointBoundControlLaw(_limb, _kj, _activationThresholds, _deactivationThresholds){ init(); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ForceFieldJointBoundControlLaw::init(){
    name = "ForceFieldJointBoundControlLaw";
    shortName = "ffjbcl";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
    if(limb != NULL){
        jointBoundMax = limb->getJointBoundMax() * CTRL_RAD2DEG;
        jointBoundMin = limb->getJointBoundMin() * CTRL_RAD2DEG;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ForceFieldJointBoundControlLaw::computeJntBoundCtrl(const RobotStatus &s, Vector &out){
    for(unsigned int i=0;i<N;i++){
        if(s.activeJoints[i]==1.0){
            out[i]  = kj[i] * (exp(-pow(s.q[i]-jointBoundMin[i], 2)/activationThresholds[i])
                              -exp(-pow(s.q[i]-jointBoundMax[i], 2)/activationThresholds[i]) );
        }
        else
            out[i] = 0.0;
    }
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ForceFieldJointBoundControlLaw::setActivationThresholds(const yarp::sig::Vector &_activationThresholds){
    mutex.wait();
    activationThresholds = _activationThresholds;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//====================================
//
//		TorqueControlLaw
//
//====================================
TorqueControlLaw::TorqueControlLaw(iDynLimb* _limb, const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds):
    ForceFieldJointBoundControlLaw(_limb, _kj, _activationThresholds, _deactivationThresholds){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void TorqueControlLaw::init(){
    ForceFieldJointBoundControlLaw::init();
    name = "TorqueControlLaw";
    shortName = "tor";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status TorqueControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    ForceFieldJointBoundControlLaw::abstractCompute(ref, s, out, feasibility);
    out += s.activeJoints * ref.torques;
    return Status();
}


//====================================
//
//		FloatControlLaw
//
//====================================
FloatControlLaw::FloatControlLaw(iCub::iDyn::iDynLimb* _limb){
    limb = _limb->asChain();
    pidN = 0;
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FloatControlLaw::FloatControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd)
: ForceFieldJointBoundControlLaw(_limb, _period, _kp, _ki, _kd){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FloatControlLaw::FloatControlLaw(iDynLimb* _limb, const Vector &_kj, const Vector &_activationThresholds, 
    const Vector &_deactivationThresholds):ForceFieldJointBoundControlLaw(_limb, _kj, _activationThresholds, _deactivationThresholds){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FloatControlLaw::FloatControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
    const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds)
: ForceFieldJointBoundControlLaw(_limb, _period, _kp, _ki, _kd, _kj, _activationThresholds, _deactivationThresholds){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void FloatControlLaw::init(){
    ForceFieldJointBoundControlLaw::init();
    name = "FloatControlLaw";
    shortName = "flo";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status FloatControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Status res = ForceFieldJointBoundControlLaw::abstractCompute(ref, s, out, feasibility);
    Vector G;
    res &= computeGravityCompensation(s, G);
    out += G;
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status FloatControlLaw::computeGravityCompensation(const RobotStatus &s, Vector &out){
    limb->setAng(CTRL_DEG2RAD * s.q);
    // joint angle vels and accs are set to zero to compute gravity torques
    limb->setDAng(zeros(N));
    limb->setD2Ang(zeros(N));

    // external wrench at end effector is supposed zero to compute gravity torques
    limb->initNewtonEuler(s.w0, s.dw0, s.d2p0, zeros(3), zeros(3));
    limb->computeNewtonEuler();
    out = s.activeJoints * limb->getTorques();
    return Status();
}


//====================================
//
//		JPositionControlLaw
//
//====================================
JPositionControlLaw::JPositionControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd): FloatControlLaw(_limb, _period, zeros(_kp.size()), _ki, zeros(_kd.size())), 
                kp(_kp), kd(_kd)
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
JPositionControlLaw::JPositionControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
    const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds)
    : FloatControlLaw(_limb, _period, zeros(_kp.size()), _ki, zeros(_kd.size()), _kj, _activationThresholds, _deactivationThresholds),
    kp(_kp), kd(_kd)
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void JPositionControlLaw::init()
{
    FloatControlLaw::init();
    name = "JPositionControlLaw";
    shortName = "jpos";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status JPositionControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility)
{
    //Vector G = limb->computeCcGravityTorques(s.d2p0, CTRL_DEG2RAD*s.q, CTRL_DEG2RAD*s.dq);
    //out = kp*(ref.q-s.q) + kd*(ref.dq-s.dq) + pid->compute(ref.q, s.q);
    Vector jb(N);
    JointBoundControlLaw::computeJntBoundCtrl(s, jb);   // joint bound ctrl

    Vector ddq_d = s.activeJoints*CTRL_DEG2RAD * (kp*(ref.q-s.q) + kd*(ref.dq-s.dq) + ref.ddq + pid->compute(ref.q, s.q));
    // inverse dynamics
    limb->setAng(CTRL_DEG2RAD*s.q);
    limb->setDAng(CTRL_DEG2RAD*s.dq);
    limb->setD2Ang(ddq_d);
    limb->computeNewtonEuler(s.w0, s.dw0, s.d2p0, zeros(3), zeros(3));
    out = limb->getTorques();
    
#define _DEBUG
#ifdef _DEBUG
    printf("kp*(ref.q-s.q): %s\n", (kp*(ref.q-s.q)).subVector(3,7).toString(1).c_str());
    printf("kd*(ref.dq-s.dq): %s\n", (kd*(ref.dq-s.dq)).subVector(3,7).toString(1).c_str());
    printf("ddq_d: %s\n", (CTRL_RAD2DEG*ddq_d.subVector(3,7)).toString(1).c_str());
    Vector h = limb->computeCcGravityTorques(s.d2p0, CTRL_DEG2RAD*s.q, CTRL_DEG2RAD*s.dq);
    printf("coriolis and gravity torques: %s\n", h.subVector(3,7).toString(2).c_str());
    printf("tau_d: %s\n", (out-h).subVector(3,7).toString(2).c_str());
#endif
    
    mergeJntBoundCtrl(jb, out);
    out *= s.activeJoints;
    feasibility = 1.0;
    return Status();
}


//====================================
//
//		PositionControlLaw
//
//====================================
PositionControlLaw::PositionControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd): FloatControlLaw(_limb, _period, _kp, _ki, _kd){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PositionControlLaw::PositionControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
    const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds)
: FloatControlLaw(_limb, _period, _kp, _ki, _kd, _kj, _activationThresholds, _deactivationThresholds){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void PositionControlLaw::init(){
    FloatControlLaw::init();
    name = "PositionControlLaw";
    shortName = "pos";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status PositionControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Vector G(N), jb(N);
    computeGravityCompensation(s, G);   // gravity compensation 
    computeJntBoundCtrl(s, jb);         // joint bound ctrl

    Matrix R_r_c = limb->getH(ref.linkNumber).submatrix(0,2,0,2);   // rotation from root to ctrl point link
    Vector ctrlPnt_r = R_r_c*ref.ctrlPoint;                         // project in root ref frame
    Vector x = limb->Pose(ref.linkNumber).subVector(0,2) + ctrlPnt_r;

    Matrix J(6, N);
    J.setSubmatrix(limb->GeoJacobian(ref.linkNumber), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0;i<N;i++)
        if(s.activeJoints[i]==0.0)
            J.setCol(i, zeros(6));

    J = contactPointJacobian(J, ctrlPnt_r);
    Matrix JT = J.submatrix(0,2,0,N-1).transposed();
    Vector pidOut = pid->compute(ref.ctrlPoint.subVector(0,2), x);
    Vector jPidOut = JT*pidOut;
    mergeJntBoundCtrl(jb, jPidOut);      // merge joint bound ctrl torque with task ctrl torque
    out = s.activeJoints*(jPidOut + G);

    // the task feasibility is the gain of JT in the direction commanded by the PID
    // if this value is too small (<1e-2) it means that the task is not feasible
    double fNorm = yarp::math::norm(pidOut);
    feasibility = (fNorm>1) ? yarp::math::norm(jPidOut)/fNorm : 1.0;

    return Status();
}


//====================================
//
//		ForceControlLaw
//
//====================================
ForceControlLaw::ForceControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
    const Vector &_kd, bool _enableMomentCtrl): 
        FloatControlLaw(_limb, _period, _kp, _ki, zeros(_kp.size())), 
        enableMomentCtrl(_enableMomentCtrl){
    init(_kd);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ForceControlLaw::ForceControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
    const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds, bool _enableMomentCtrl)
    : FloatControlLaw(_limb, _period, _kp, _ki, zeros(_kp.size()), _kj, _activationThresholds, _deactivationThresholds),
    enableMomentCtrl(_enableMomentCtrl){
    init(_kd);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ForceControlLaw::init(const Vector &_kd){
    FloatControlLaw::init();
    name = "ForceControlLaw";
    shortName = "for";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
    // PID kd are set to zero because the derivative term is computed using joint velocities
    FloatControlLaw::setKd(Vector(pidN,0.0));
    kd = _kd;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ForceControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Vector G(N), jb(N);
    computeGravityCompensation(s, G);   // gravity compensation
    computeJntBoundCtrl(s, jb);         // joint bound ctrl

    Matrix R_r_c  = limb->getH(ref.linkNumber).submatrix(0,2,0,2);  // rotation from root to ctrl point link
    Vector ctrlPnt_r = R_r_c*ref.ctrlPoint;
    
    Matrix J(6, N);
    J.setSubmatrix(limb->GeoJacobian(ref.linkNumber), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0;i<N;i++)
        if(s.activeJoints[i]==0.0)
            J.setCol(i, zeros(6));

    J = contactPointJacobian(J, ctrlPnt_r);
    Matrix JT = J.transposed();
    Vector pidOut = -1.0 * pid->compute(ref.wrench, s.wrench) - ref.wrench;
    if(!enableMomentCtrl){
        JT = JT.submatrix(0,N-1,0,2);     // remove the rotational part
        pidOut = pidOut.subVector(0,2);
    }

    // the PID doesn't compute the derivative term
    Vector ctrlTorque = JT*pidOut - kd*s.dq;
    mergeJntBoundCtrl(jb, ctrlTorque);      // merge joint bound ctrl torque with task ctrl torque
    out = s.activeJoints * (ctrlTorque + G);
    
    // the task feasibility is the gain of JT in the direction commanded by the PID
    // if this value is too small (<1e-2) it means that the task is not feasible
    double fNorm = yarp::math::norm(pidOut);
    feasibility = (fNorm>1) ? yarp::math::norm(JT*pidOut)/fNorm : 1.0;

    return Status();
} 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ForceControlLaw::setKd(const Vector &_kd){
    if(_kd.size()!=kd.size())
        return Status("Error while setting param Kd, expected size: "+toString(kd.size())+"; found size: "+toString(_kd.size()));
    mutex.wait();
    kd = _kd;
    mutex.post();
    return Status();
}


//====================================
//
//		ParallelControlLaw
//
//====================================
ParallelControlLaw::ParallelControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd, const Vector &_kf, const Vector &_kv): FloatControlLaw(_limb, _period, _kf, _ki, zeros(_kf.size())), 
                kd(_kd), kp(_kp), kv(_kv){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ParallelControlLaw::ParallelControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
        const Vector &_kf, const Vector &_kv, const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds)
        : FloatControlLaw(_limb, _period, _kf, _ki, zeros(_kf.size()), _kj, _activationThresholds, _deactivationThresholds),
    kd(_kd), kp(_kp), kv(_kv){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ParallelControlLaw::init(){
    FloatControlLaw::init();
    name = "ParallelControlLaw";
    shortName = "par";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
    int cmdSize = ParallelControlLawCommandSize - commandList.size();
    commandList.insert(commandList.end(), ParallelControlLawCommand_s, ParallelControlLawCommand_s+cmdSize);
    commandDesc.insert(commandDesc.end(), ParallelControlLawCommand_desc, ParallelControlLawCommand_desc+cmdSize);
    setKd(Vector(pidN,0.0));
    setKp2(Vector(N, 0.0));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Vector G(N), jb(N);
    computeGravityCompensation(s, G);   // gravity compensation
    computeJntBoundCtrl(s, jb);         // joint bound ctrl

    Matrix R_r_c  = limb->getH(ref.linkNumber).submatrix(0,2,0,2);       // rotation from root to ctrl point link
    Vector ctrlPnt_r = R_r_c*ref.ctrlPoint;
    Matrix R_r_c2 = limb->getH(ref.linkNumber2).submatrix(0,2,0,2);       // rotation from root to ctrl point link
    Vector ctrlPnt2_r = R_r_c2*ref.ctrlPoint2;
    Vector x = limb->Pose(ref.linkNumber).subVector(0,2) + ctrlPnt_r;
    
    Matrix J(6, N), J2(6, N);   // J=jacobian of pos ctrl pnt, J2=jacobian of force ctlr pnt
    J.setSubmatrix(limb->GeoJacobian(ref.linkNumber), 0, 0);
    J2.setSubmatrix(limb->GeoJacobian(ref.linkNumber2), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0; i<N; i++){
        if(s.activeJoints[i]==0.0){
            J.setCol(i, zeros(6) );
            J2.setCol(i, zeros(6) );
        }
    }
    J = contactPointJacobian(J, ctrlPnt_r);
    J2 = contactPointJacobian(J2, ctrlPnt2_r);
    J  = J.submatrix(0,2,0,N-1);
    J2 = J2.submatrix(0,2,0,N-1);
    
    Vector dx = CTRL_DEG2RAD*(J*s.dq);

    Vector robForce = s.wrench.subVector(0,2);
    Vector F_f = -1.0 * pid->compute(ref.wrench.subVector(0,2), robForce);    // force PI
    F_f -= ref.wrench.subVector(0,2);                   // feedforward force
    Vector F_p = kp*(ref.x-x);                           // position P
    //F_p += kv*(ref.dx.subVector(0,2)); //-dx);        // velocity
    out = F_p*J + F_f*J2 - kd*s.dq;                    // joint position D

    //Vector F1 = -1.0 * pid->compute(ref.wrench.subVector(0,2), robForce);    // force PI
    //Vector F2 = -0.5*ref.wrench.subVector(0,2); 
    //Vector F3 = kp*(ref.x.subVector(0,2)-x);      // position P + feedforward force
    //Vector JF1 = JT*F1;
    //Vector JF2 = JT*F2;
    //Vector JF3 = JT*F3;
    //static int counter=0;
    //int jnt = 6;
    //if(counter%20 == 0)
    //    printf("force PI: %.3f\t +force ff: %.3f\t +posP: %.3f\t posD: %.3f\t tot: %.3f\n", JF1(jnt), JF2(jnt), JF3(jnt), kd(jnt)*dq(jnt), out(jnt));
    //counter++;

    mergeJntBoundCtrl(jb, out);                 // merge joint bound ctrl torque with task ctrl torque
    out = s.activeJoints*(out + G);             // position D + gravity compensation

    // the task feasibility is the gain of JT in the direction commanded by the PID
    // if this value is too small (<1e-2) it means that the task is not feasible
    double fNorm = yarp::math::norm(F_f);
    feasibility = (fNorm>1) ? yarp::math::norm(F_f*J2)/fNorm : 1.0;

    // MONITOR DATA
    monitor.resize(3+3+3+3);
    unsigned int index = 0;
    monitor.setSubvector(index, dx); index+=3;
    monitor.setSubvector(index, F_f); index+=3;
    monitor.setSubvector(index, F_p); index+=3;
    monitor.setSubvector(index, ref.x*1e2); index+=3;

    return Status();
} 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::setParameter(const string &key, const Vector& value){
    if(key == "Kd")
        return setKd(value);
    Status s = FloatControlLaw::setParameter(key, value);
    if(s)  return s;
    if(key == "Kf")
        return setKf(value);
    return s && Status("Error while trying to set the parameter "+key+": unknown parameter name");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::setKp(const Vector &_kp) {
    if(_kp.size()!=3)
        return Status(false, "Error setting kp: wrong size");
    mutex.wait();
    kp=_kp;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::setKp2(const Vector &_kp2) {
    if(_kp2.size()!=N)
        return Status(false, "Error setting kp2: wrong size");
    mutex.wait();
    kp2=_kp2;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::setKv(const Vector &_kv) {
    if(_kv.size()!=3)
        return Status(false, "Error setting kv: wrong size");
    mutex.wait();
    kv=_kv;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::setKd(const Vector &_kd){
    if(_kd.size()!=kd.size())
        return Status("Error while setting param Kd, expected size: "+toString(kd.size())+"; found size: "+toString(_kd.size()));
    mutex.wait();
    kd = _kd;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::getParameter(const string &key, Vector &v){
    if(key=="Kd"){
        v = getKd();
        return Status();
    }
    Status s = FloatControlLaw::getParameter(key, v);
    if(s) return s;
    if(key=="Kf")
        v = getKf();
    else
        return Status("Error while trying to set the parameter "+key+": unknown parameter name");
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ParallelControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kf:    reply.addString(getKf().toString(3).c_str()); break;
        case get_kv:    reply.addString(getKv().toString(3).c_str()); break;
        case get_kp2:   reply.addString(getKp2().toString(3).c_str()); break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s) break;
                switch(cmdId){
                    case set_kf:
                        if(v.size()==1)
                            s = setKf(Vector(pidN, v[0]));
                        else
                            s = setKf(v);
                        break;
                    case set_kd:    // reimplement "set kd" because kd dimension isn't pidN, but N
                        if(v.size()==1)
                            s = setKd(Vector(N,v[0]));
                        else
                            s = setKd(v);
                        break;
                    case set_kv:
                        if(v.size()==1)
                            s = setKv(Vector(3, v[0]));
                        else
                            s = setKv(v);
                        break;
                    case set_kp2:
                        if(v.size()==1)
                            s = setKp2(Vector(N, v[0]));
                        else
                            s = setKp2(v);
                        break;
                    default:
                        return JointBoundControlLaw::respond(cmdId, params, reply);
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}


//====================================
//
//		DynParallelControlLaw
//
//====================================
DynParallelControlLaw::DynParallelControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd, const Vector &_kf, const Vector &_kv): ParallelControlLaw(_limb, _period, _kp, _ki, _kd, _kf, _kv){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
DynParallelControlLaw::DynParallelControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
        const Vector &_kf, const Vector &_kv, const Vector &_kj, const Vector &_activationThresholds, const Vector &_deactivationThresholds)
        : ParallelControlLaw(_limb, _period, _kp, _ki, _kd, _kf, _kv, _kj, _activationThresholds, _deactivationThresholds){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DynParallelControlLaw::init(){
    ParallelControlLaw::init();
    name = "DynParallelControlLaw";
    shortName = "dpar";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;

    jb = ddq_p = ddq_f = zeros(N);
    R_r_c = eye(3);
    f = fd = ctrlPnt_r = x = dx = ddx_f = ddx_p = zeros(3);

    tempJ = tempdJ = zeros(6,N);
    Jl_p = Jl_f = dJl_p = dJl_f = zeros(3,N);
    Jl_p_pinv = Jl_f_pinv = zeros(N,3);
    
    z6 = zeros(6); 
    z3 = zeros(3);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DynParallelControlLaw::computeJacobians(unsigned int linkNumber, const Vector &ctrlPoint, 
    const Vector &activeJoints, const Vector &dqRad, Matrix &Jl, Matrix &dJl)
{
    R_r_c  = limb->getH(linkNumber).submatrix(0,2,0,2);       // rotation from root to ctrl point link
    ctrlPnt_r = R_r_c*ctrlPoint;

    tempJ.zero(); tempdJ.zero();
    tempJ.setSubmatrix(limb->GeoJacobian(linkNumber), 0, 0);
    tempdJ.setSubmatrix(limb->DJacobian(linkNumber, dqRad), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0; i<N; i++)
        if(activeJoints[i]==0.0)
            tempJ.setCol(i, z6);
    contactPointJacobian(ctrlPnt_r, tempJ);
    contactPointJacobian(ctrlPnt_r, tempdJ);
    Jl  = tempJ.submatrix(0,2,0,N-1);
    dJl = tempdJ.submatrix(0,2,0,N-1);
    //pinvDamped(Jl, Jl_pinv, sv_Jl, pinvDamp);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status DynParallelControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Vector dqRad = CTRL_DEG2RAD * s.dq; // joint velocities in rad/sec
    fd = ref.wrench.subVector(0,2);
    f = s.wrench.subVector(0,2);

    computeJntBoundCtrl(s, jb);         // joint bound ctrl
    limb->setAng(CTRL_DEG2RAD * s.q);   // should be already set, but just in case
    limb->setDAng(dqRad);

    computeJacobians(ref.linkNumber, ref.ctrlPoint, s.activeJoints, dqRad, Jl_p, dJl_p);
    computeJacobians(ref.linkNumber2, ref.ctrlPoint2, s.activeJoints, dqRad, Jl_f, dJl_f);

    x = (limb->getH(ref.linkNumber) * cat(ref.ctrlPoint, 1.0)).subVector(0,2);
    dx = Jl_p*dqRad; //linear velocity of position ctlr pnt

    ddx_f = -1.0 * pid->compute(fd,f);                  // force PI
    ddx_p = ref.ddx + kv*(ref.dx-dx) + kp*(ref.x-x);    // position PD
    y_f = ddx_f - dJl_f*dqRad;
    y_p = ddx_p - dJl_p*dqRad;

    // compute joint acceleration bounds and convert them from deg to rad
    //computeAccBounds(jointBoundMin, jointBoundMax, ref.dqMax, s.q, s.dq, activationThresholds, 1e-2, ddqL, ddqU);
    //computeAccBounds(jointBoundMin, jointBoundMax, ref.dqMax, s.q, s.dq, activationThresholds, 1e-3*period, ddqL, ddqU);
    //ddqL *= CTRL_DEG2RAD*s.activeJoints;
    //ddqU *= CTRL_DEG2RAD*s.activeJoints;
    //// temporarely increase the acc bounds
    //ddqL *= 10.0;
    //ddqU *= 10.0;
    // these accelerations are already expressed in rad/s^2
    /*double tollerance = ref.pinvDamp;
    unsigned int dampF = pinvBounded(Jl_f, y_f, ddqL,       ddqU,       Jl_f_pinv, ddq_f, tollerance);
    unsigned int dampP = pinvBounded(Jl_p, y_p, ddqL-ddq_f, ddqU-ddq_f, Jl_p_pinv, ddq_p, tollerance);*/

    pinvDamped(Jl_f, Jl_f_pinv, ref.pinvDamp);
    pinvDamped(Jl_p, Jl_p_pinv, ref.pinvDamp);
    N_f = eye(N,N) - Jl_f_pinv*Jl_f;
    ddq_f = Jl_f_pinv*y_f;
    ddq_p = Jl_p_pinv*y_p;

    // inverse dynamics
    limb->setD2Ang(ddq_p+ddq_f);
    limb->computeNewtonEuler(s.w0, s.dw0, s.d2p0, z3, z3);
    out = limb->getTorques();
    
    out -= fd*Jl_f;                         // force feedforward
    out -= kd*s.dq;                         // joint damping

    // secondary task (with dynamic decoupling)
    limb->setDAng(zeros(s.dq.size()));
    limb->setD2Ang(kp2*(ref.q-s.q));
    limb->computeNewtonEuler(z3, z3, z3, z3, z3);
    out += N_f*limb->getTorques();
    
    mergeJntBoundCtrl(jb, out);             // merge joint bound ctrl torque with task ctrl torque
    out *= s.activeJoints;                  // remove blocked joints

    // task feasibility is the percentage of task space acceleration that can be achieved
    // i.e. 1 - (||ddx_d - ddx|| / ||ddx_d||)
    double norm_ddx_p = math::norm(ddx_p);
    double feasibility_p = norm_ddx_p>1e-2 ? (1.0 -  math::norm((eye(3)-Jl_p*Jl_p_pinv)*y_p)/norm_ddx_p) : 1.0;
    double norm_ddx_f = math::norm(ddx_f);
    double feasibility_f = norm_ddx_f>1e-2 ? (1.0 -  math::norm((eye(3)-Jl_f*Jl_f_pinv)*y_f)/norm_ddx_f) : 1.0;
    feasibility = 0.5*(feasibility_f+feasibility_p);

#ifndef _DEBUG
    //#define _DEBUG
#endif
#ifdef _DEBUG
    //printf("Fd: %s\n", fd.toString(1).c_str());
    //if(norm_ddx_p != 0.0){
    //    Vector vTerm = kv*(ref.dx-dx);
    //    printf("Vel percentage contribution: %.3f\n", math::norm(vTerm) / (math::norm(vTerm)+math::norm(ddx_p-vTerm)));
    //    printf("Acc percentage contribution: %.3f\n", math::norm(ref.ddx) / (math::norm(ref.ddx)+math::norm(ddx_p-ref.ddx)));
    //}
    //printf("x:      %s\n", x.toString(3).c_str());
    //printf("xLink:  %s\n", limb->Pose(ref.linkNumber).subVector(0,2).toString(3).c_str());
    //printf("ctrlPnt:%s\n", ctrlPnt_r.toString(3).c_str());
    //printf("ddq_f: %s rad/sec^2\n", (1.0*ddq_f).toString(1).c_str());
    //printf("ddq_p: %s rad/sec^2\n", (1.0*ddq_p).toString(1).c_str());
    //printf("damping F: %d\t damping P: %d\n", dampF, dampP);
    printf("feasibility f: %f\n, feasibility p: %f\n", feasibility_f*1e2, feasibility_p*1e2);
    printf("Cartesian J_pinv gains: %.3f %.3f %.3f\n", math::norm(Jl_p_pinv.getCol(0)), math::norm(Jl_p_pinv.getCol(1)), math::norm(Jl_p_pinv.getCol(2)));
    //printf("kp*(xRef-x): %s\n", (kp*(ref.x-x)).toString(3).c_str());
    //printf("kv*(dxRef-dx): %s\n", (kv*(dxRef-dx)).toString(3).c_str()); 
    //printf("xRef:   %s cm\n", (1e2*ref.x).toString(3).c_str());
    //printf("x:   %s cm\n", (1e2*x).toString(3).c_str());
    //printf("dxRef:  %s cm/s\n", (100*dxRef).toString(3).c_str());
    //printf("ddxRef: %s cm/s2\n", (100*ddxRef).toString(3).c_str());
    //printf("ddx_p: %s\n", ddx_p.toString(3).c_str());
    //printf("ddx_f: %s\n", ddx_f.toString(3).c_str());
    //printf("dJ*dq: %s\n", (dJl_p*dqRad).toString(3).c_str());
    /*Vector tau_acc = (out - s.activeJoints*limb->computeCcGravityTorques(s.d2p0));
    printf("tau_acc: %s\n", tau_acc.toString(3).c_str());*/
    //printf("Mass matrix gain= %.1fe-3\n", 1e3*math::norm(tau_acc)/math::norm(ddq_p+ddq_f));
    printf("  tau0: %s\n", (kp2*(ref.q-s.q)).toString(3).c_str());
    printf("N*tau0: %s\n", (N_f*(kp2*(ref.q-s.q))).toString(3).c_str());
#endif

    return Status();
}


//====================================
//
//		ContactControlLaw
//
//====================================
ContactControlLaw::ContactControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, 
                const Vector &_kd, const Vector &_kf, const Vector &_kv, double _trajTime, 
                double _forcePriority, const Vector _kc)
                : ForceFieldJointBoundControlLaw(_limb, _period, _kf, _ki, zeros(_ki.size())), 
                kp(_kp), kv(_kv), kd(_kd), trajTime(_trajTime), forcePriority(_forcePriority), kc(_kc)
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ContactControlLaw::ContactControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_ki, const Vector &_kd, 
        const Vector &_kf, const Vector &_kv, double _trajTime, double _forcePriority, const Vector _kc, const Vector &_kj, 
        const Vector &_activationThresholds, const Vector &_deactivationThresholds)
        : ForceFieldJointBoundControlLaw(_limb, _period, _kf, _ki, zeros(_ki.size()), _kj, _activationThresholds, _deactivationThresholds),
        kp(_kp), kv(_kv), kd(_kd), trajTime(_trajTime), forcePriority(_forcePriority), kc(_kc)
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactControlLaw::init()
{
    ForceFieldJointBoundControlLaw::init();
    name = "ContactControlLaw";
    shortName = "cont";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
    int cmdSize = ContactControlLawCommandSize - commandList.size();
    commandList.insert(commandList.end(), ContactControlLawCommand_s, ContactControlLawCommand_s+cmdSize);
    commandDesc.insert(commandDesc.end(), ContactControlLawCommand_desc, ContactControlLawCommand_desc+cmdSize);
    monoTask = true;

    monitor = zeros(50);
    kp2 = zeros(N);
    jb = zeros(N);
    R_r_c = eye(3);
    C_r = C = dC = zeros(3);
    J = zeros(7, N);
    Jlink = dJlink = Jc = zeros(6, N);
    Jb = Jd = dJb = dJc = Jc_l = Jc_a = zeros(3, N);

    trajGen_B = new minJerkRefGen(3, period*1e-3, trajTime);
    trajGen_C = new minJerkTrajGen(3, period*1e-3, trajTime);
    
    z6 = zeros(6); 
    z3 = zeros(3);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::reset(const CtrlRef &ref, const RobotStatus &s)
{
    Status ss = ForceFieldJointBoundControlLaw::reset(ref, s);
    if(!ss) return ss;
    referenceChanged(ref, s);   // reset minJerkTrajGen
    return ss;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactControlLaw::referenceChanged(const CtrlRef &ref, const RobotStatus &s)
{
    // reset minJerkTrajGen
    limb->setAng(CTRL_DEG2RAD * s.q);           // should be already set, but just in case
    H_r_c  = limb->getH(ref.linkNumber2);       // rototranslation from root to ctrl point link
    B = (H_r_c * cat(ref.ctrlPointRef, 1.0)).subVector(0,2);
    C = (H_r_c * cat(ref.ctrlPoint2, 1.0)).subVector(0,2);
    printf("ContactControlLaw: reset minJerkRefGen to %s\n", B.toString(2).c_str());
    trajGen_B->init(B);
    trajGen_C->init(C);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactControlLaw::computeJacobians(const CtrlRef &ref, const RobotStatus &s)
{
    dqRad = CTRL_DEG2RAD * s.dq;                // joint velocities in rad/sec
    limb->setAng(CTRL_DEG2RAD * s.q);           // should be already set, but just in case
    limb->setDAng(dqRad);
    H_r_c  = limb->getH(ref.linkNumber2);       // rototranslation from root to ctrl point link
    R_r_c  = H_r_c.submatrix(0,2,0,2);          // rotation from root to ctrl point link

    B = (H_r_c * cat(ref.ctrlPointRef, 1.0)).subVector(0,2);
    C = (H_r_c * cat(ref.ctrlPoint2, 1.0)).subVector(0,2);
    C_r = R_r_c*ref.ctrlPoint2;
    D = ref.xRef;

    Jlink.zero(); dJlink.zero();
    Jlink.setSubmatrix(limb->GeoJacobian(ref.linkNumber2), 0, 0);
    dJlink.setSubmatrix(limb->DJacobian(ref.linkNumber2, dqRad), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0; i<N; i++)
        if(s.activeJoints[i]==0.0)
            Jlink.setCol(i, z6);

    Jc  = contactPointJacobian(Jlink, C_r);
    Jc_l = Jc.submatrix(0,2,0,N-1);
    Jc_a = Jc.submatrix(3,5,0,N-1);
    dJc = contactPointJacobian(dJlink, C_r);
    dJc_l = dJc.submatrix(0,2,0,N-1);
    dJc_a = dJc.submatrix(3,5,0,N-1);
    Jb  = Jc_l - crossProductMatrix(B-C)*Jc_a;
    dJb = dJc_l - crossProductMatrix(B-C)*dJc_a;
    Jd  = -1.0*Jc_l + crossProductMatrix(D-C)*Jc_a;
    Jf  = s.wrenchSmooth.subVector(0,2)*Jc_l;
    
    J.setSubmatrix(Jb,0,0);
    if(monoTask)
        J.setSubmatrix(zeros(3,N),3,0);
    else
        J.setSubmatrix(-1.0*Jd,3,0);
    J.setRow(6, forcePriority*Jf);
    pinvDamped(J, J_pinv, ref.pinvDamp);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility)
{
    if(ref.linkNumber != ref.linkNumber2)
        printf("Error ctrlPoint and contact point are on different links (%d, %d)\n", ref.linkNumber, ref.linkNumber2);

    computeJacobians(ref, s);

    fd = ref.wrench.subVector(0,2);                 // desired contact force
    f = s.wrench.subVector(0,2);                    // measured contact force
    fDir = versor(s.wrenchSmooth.subVector(0,2));   // use the smooth force to compute the force direction
    computeJntBoundCtrl(s, jb);                     // joint bound ctrl

    // linear velocities
    dB = Jb*dqRad;
    dC = Jc_l*dqRad;
    // Minimum jerk for B, C
    if(monoTask)
        trajGen_B->computeNextValues(B, D);
    else
        trajGen_B->computeNextValues(B, C);
    trajGen_C->computeNextValues(D);
    Bref = trajGen_B->getPos();
    Cref = trajGen_C->getPos();
    // desired accelerations for B, C
    ddBd = trajGen_B->getAcc() + kv*(trajGen_B->getVel()-dB) + kp*(Bref-B);
    ddCd = trajGen_C->getAcc() + kv*(trajGen_C->getVel()-dC) + kp*(Cref-C);
    ddx_f = dot(fDir, -1.0*pid->compute(fd,f));   // force PI

    yB = ddBd - dJb*dqRad;
    if(monoTask)
        yC = zeros(3);
    else
        yC = ddCd - dJc_l*dqRad;
    y = cat(yB,cat(yC, forcePriority*ddx_f));
    ddq = kc*(J_pinv*y);

    // inverse dynamics
    limb->setD2Ang(ddq);
    limb->computeNewtonEuler(s.w0, s.dw0, s.d2p0, z3, z3);
    out = limb->getTorques();
    
    Matrix N_f = eye(N,N) - J_pinv*J;
    // secondary task (with dynamic decoupling)
    limb->setDAng(zeros(s.dq.size()));
    limb->setD2Ang(kp2*(ref.q-s.q));
    limb->computeNewtonEuler(z3, z3, z3, z3, z3);
    Vector tao_0 = N_f*limb->getTorques();
    out += tao_0;
    
    out -= fd*Jc_l;                         // force feedforward
    out -= kd*s.dq;                         // joint damping
    mergeJntBoundCtrl(jb, out);             // merge joint bound ctrl torque with task ctrl torque
    out *= s.activeJoints;                  // remove blocked joints

    // task feasibility is the percentage of task space acceleration that can be achieved
    // i.e. 1 - (||ddx_d - ddx|| / ||ddx_d||)
    double norm_y = norm(y);
    feasibility = norm_y>1e-2 ? (1.0 -  norm(y-J*ddq)/norm_y) : 1.0;
    ddB = Jb*ddq;
    ddC = Jc_l*ddq;
    dwC = Jc_a*ddq;
    ddf = dot(Jf,ddq);

    sendMonitorData();

//#define _DEBUG
#ifdef _DEBUG
    int m = Jb.rows(), n = Jb.cols(), k = m<n?m:n;
	Matrix U(m,k), V(n,k);
	Vector Sdiag(k), Sinv(k);
	SVD(Jb, U, Sdiag, V);
    double damp2 = ref.pinvDamp*ref.pinvDamp;
	for (int c=0;c<k; c++)
		Sinv(c) = Sdiag(c) / (Sdiag(c)*Sdiag(c) + damp2);
		
	printf("tao0= %s\n", tao_0.toString(2).c_str());
    printf("U:\n%s\n", U.toString(2).c_str());
    printf("U^T*ddBd:\n%s\n", (ddBd*U).toString(2).c_str());
    printf("S:    %s\n", Sdiag.toString(2).c_str());
    printf("Sinv: %s\n", Sinv.toString(2).c_str());
    printf("V^T:\n%s\n", V.submatrix(3,7,0,2).transposed().toString(2).c_str());

    printf("ddq: %s\n", ddq.toString(1).c_str());
    printf("ddx   due to ddq[3]: %s\n", J.getCol(3).subVector(0,2).toString(2).c_str());
    printf("ddx   due to ddq[4]: %s\n", J.getCol(4).subVector(0,2).toString(2).c_str());
    printf("ddx   due to ddq[6]: %s\n", J.getCol(6).subVector(0,2).toString(2).c_str());
    printf("ddx   due to ddq[7]: %s\n", J.getCol(7).subVector(0,2).toString(2).c_str());

    printf("ddq   due to ddx: %s\n", J_pinv.getCol(0).subVector(3,7).toString(2).c_str());
    printf("ddq   due to ddy: %s\n", J_pinv.getCol(1).subVector(3,7).toString(2).c_str());
    printf("ddq   due to ddz: %s\n", J_pinv.getCol(2).subVector(3,7).toString(2).c_str());

    /*printf("ddx_f due to ddq[7]: %.2f\n", J(6,7));
    printf("B:    %s\n", B.toString(2).c_str());
    printf("Bref: %s\n", Bref.toString(2).c_str());
    printf("C:    %s\n", C.toString(2).c_str());
    printf("Cref: %s\n", Cref.toString(2).c_str());
    printf("D:    %s\n", D.toString(2).c_str());*/

    printf("ddBd:     %s\n", ddBd.toString(2).c_str());
    /*printf("ddB-ddBd: %s\n", (ddB-ddBd).toString(2).c_str());
    printf("ddCd:     %s\n", ddCd.toString(2).c_str());
    printf("ddC-ddCd: %s\n", (ddC-ddCd).toString(2).c_str());
    printf("dwC:      %s\n", dwC.toString(2).c_str());
    
    printf("||ddBd|| VS ||ddBd-ddB||:  %.1f  VS  %.1f\n", norm(ddBd), norm(ddBd-ddB));
    printf("||ddCd|| VS ||ddCd-ddC||:  %.1f  VS  %.1f\n", norm(ddCd), norm(ddCd-ddC));
    printf("  ddx_f  VS   ddxf-ddx  :  %.1f  VS  %.1f\n", ddx_f, ddx_f-ddf);
    printf("feasibility:   %.1f\n", feasibility*1e2);
    
    printf("tau: %s\n", out.toString(2).c_str());*/

    printf("B:         %s\n", B.toString(2).c_str());
    printf("D:         %s\n", D.toString(2).c_str());
    printf("B ref pos: %s\n", Bref.toString(2).c_str());
    printf("B ref vel: %s\n", trajGen_B->getVel().toString(2).c_str());
    printf("B ref acc: %s\n", trajGen_B->getAcc().toString(2).c_str());
#endif

    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactControlLaw::sendMonitorData()
{
    // MONITOR DATA
    monitor.zero();
    unsigned int index = 0;
    // 0-8
    monitor.setSubvector(index, B); index+=3;
    monitor.setSubvector(index, C); index+=3;
    monitor.setSubvector(index, D); index+=3;
    // 9-14
    monitor.setSubvector(index, Bref); index+=3;
    monitor.setSubvector(index, Cref); index+=3;
    // 15-21
    monitor.setSubvector(index, ddBd); index+=3;
    monitor.setSubvector(index, ddCd); index+=3;
    monitor[index++] = ddx_f;
    // 22-28
    monitor.setSubvector(index, ddB); index+=3;
    monitor.setSubvector(index, ddC); index+=3;
    monitor[index++] = ddf;
    // 29-31
    monitor.setSubvector(index, dwC); index+=3;
    // 32-41
    monitor.setSubvector(index, ddq); index+=N;
    // 42-47
    monitor.setSubvector(index, trajGen_B->getVel()); index+=3;
    monitor.setSubvector(index, trajGen_B->getAcc()); index+=3;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setParameter(const string &key, const Vector& value){
    if(key == "Kd")
        return setKd(value);
    Status s = ForceFieldJointBoundControlLaw::setParameter(key, value);
    if(s)  return s;
    if(key == "Kf")
        return setKf(value);
    return s && Status("Error while trying to set the parameter "+key+": unknown parameter name");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setKp(const Vector &_kp) {
    if(_kp.size()!=3)
        return Status(false, "Error setting kp: wrong size");
    mutex.wait();
    kp=_kp;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setKp2(const Vector &_kp2) {
    if(_kp2.size()!=N)
        return Status(false, "Error setting kp2: wrong size");
    mutex.wait();
    kp2=_kp2;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setKv(const Vector &_kv) {
    if(_kv.size()!=3)
        return Status(false, "Error setting kv: wrong size");
    mutex.wait();
    kv=_kv;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setKd(const Vector &_kd){
    if(_kd.size()!=kd.size())
        return Status("Error while setting param Kd, expected size: "+toString(kd.size())+"; found size: "+toString(_kd.size()));
    mutex.wait();
    kd = _kd;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setKc(const Vector &_kc){
    if(_kc.size()!=kc.size())
        return Status("Error while setting param Kc, expected size: "+toString(kc.size())+"; found size: "+toString(_kc.size()));
    mutex.wait();
    kc = _kc;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setContTrajTime(double _trajTime){
    if(_trajTime<=0.0)
        return Status("Error while setting contact trajectory time: non positive value "+toString(_trajTime));
    mutex.wait();
    trajTime = _trajTime;
    trajGen_B->setT(trajTime);
    trajGen_C->setT(trajTime);
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setForcePriority(double fp){
    if(fp<0.0)
        return Status("Error while setting force priority: negative value "+toString(fp));
    mutex.wait();
    forcePriority = fp;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::setMono(bool mono){
    mutex.wait();
    if(monoTask!= mono)
    {
        monoTask = mono;
        trajGen_B->init(B);
        trajGen_C->init(C);
    }
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::getParameter(const string &key, Vector &v){
    if(key=="Kd"){
        v = getKd();
        return Status();
    }
    Status s = ForceFieldJointBoundControlLaw::getParameter(key, v);
    if(s) return s;
    if(key=="Kf")
        v = getKf();
    else
        return Status("Error while trying to set the parameter "+key+": unknown parameter name");
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status ContactControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kf:    reply.addString(getKf().toString(3).c_str()); break;
        case get_kv:    reply.addString(getKv().toString(3).c_str()); break;
        case get_kp2:   reply.addString(getKp2().toString(3).c_str()); break;
        case get_kc:    reply.addString(getKc().toString(1).c_str()); break;
        case get_cont_traj_time: reply.addDouble(getContTrajTime()); break;
        case get_force_priority: reply.addDouble(getForcePriority()); break;
        case get_mono:  reply.addDouble(getMono()?1.0:0.0); break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s) break;
                switch(cmdId){
                    case set_kf:
                        if(v.size()==1)
                            s = setKf(Vector(pidN, v[0]));
                        else
                            s = setKf(v);
                        break;
                    case set_kd:    // reimplement "set kd" because kd dimension isn't pidN, but N
                        if(v.size()==1)
                            s = setKd(Vector(N,v[0]));
                        else
                            s = setKd(v);
                        break;
                    case set_kv:
                        if(v.size()==1)
                            s = setKv(Vector(3, v[0]));
                        else
                            s = setKv(v);
                        break;
                    case set_kp2:
                        if(v.size()==1)
                            s = setKp2(Vector(N, v[0]));
                        else
                            s = setKp2(v);
                        break;
                    case set_kc:
                        if(v.size()==1)
                            s = setKc(Vector(N, v[0]));
                        else
                            s = setKc(v);
                        break;
                    case set_cont_traj_time:
                        if(v.size()>0)
                            s = setContTrajTime(v[0]);
                        else
                            s.addErrMsg("Error while trying to set contact trajectory time: missing value!");
                        break;
                    case set_force_priority:
                        if(v.size()>0)
                            s = setForcePriority(v[0]);
                        else
                            s.addErrMsg("Error while trying to set force priority: missing value!");
                        break;
                    case set_mono:
                        if(v.size()>0)
                            s = setMono(v[0]!=0.0);
                        else
                            s.addErrMsg("Error while trying to set mono: missing value!");
                        break;
                    default:
                        return JointBoundControlLaw::respond(cmdId, params, reply);
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}

//====================================
//
//		PressureControlLaw
//
//====================================
PressureControlLaw::PressureControlLaw(iDynLimb* _limb, int _period, const Vector &_kp)
    : IControlLaw(_limb, _period, _kp, zeros(_kp.size()), zeros(_kp.size())), kp(_kp){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void PressureControlLaw::init(){
    name = "PressureControlLaw";
    shortName = "pre";
    mutex.setName(name+"Mutex");
    ctrlMode = VEL_CTRL_MODE;

    dq = zeros(N);
    z6 = zeros(6);
    tempJ = zeros(6, N);            // jacobian
    Jl = zeros(3, N);               // Jl= linear part of jacobian
    Jl_pinv = zeros(N, 3);          // jacobian pseudo-inverse
    sv_Jl = zeros(min<int>(3,N));   // singular values of jacobian
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void PressureControlLaw::computeJacobians(unsigned int linkNumber, const Vector &ctrlPoint, const Vector &activeJoints, 
    Matrix &Jl, Matrix &Jl_pinv, Vector &sv_Jl, double pinvDamp)
{
    Matrix R_r_c  = limb->getH(linkNumber).submatrix(0,2,0,2);       // rotation from root to ctrl point link
    Vector ctrlPnt_r = R_r_c*ctrlPoint;

    tempJ.zero();
    tempJ.setSubmatrix(limb->GeoJacobian(linkNumber), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0; i<N; i++)
        if(activeJoints[i]==0.0)
            tempJ.setCol(i, z6);
    contactPointJacobian(ctrlPnt_r, tempJ);
    Jl  = tempJ.submatrix(0,2,0,N-1);
    pinvDamped(Jl, Jl_pinv, sv_Jl, pinvDamp);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status PressureControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility){
    Vector dqRad = s.dq*CTRL_DEG2RAD;
    /*computeJacobians(ref.linkNumber, ref.ctrlPoint, s.activeJoints, Jl, Jl_pinv, sv_Jl, ref.pinvDamp);
    Vector n = s.contactPressure * limb->getH(ref.linkNumber).submatrix(0,2,0,2) * s.contactNormal;
    Vector pidOut = pid->compute(zeros(3), n);
    out = Jl_pinv * pidOut;*/
    Vector n, qVel;
    out = zeros(N);
    //printf("%d contacts\n", s.contactList.size());
    for(unsigned int i=0; i<s.contactList.size(); i++)
    {
        const skinContact *c = &(s.contactList[i]);
        if(norm(c->getNormalDir())!=0.0)
        {
            computeJacobians(c->getLinkNumber()+TORSO_DOF, c->getCoP(), s.activeJoints, Jl, Jl_pinv, sv_Jl, ref.pinvDamp);
            n = c->getPressure() * limb->getH(c->getLinkNumber()+TORSO_DOF).submatrix(0,2,0,2)*c->getNormalDir();
            //qVel = -1.0*Jl_pinv*(kp*n);
            qVel = -1.0*Jl.transposed()*(kp*n);
            out += qVel;
//#define _DEBUG
#ifdef _DEBUG
            printf("sv: %s\n", sv_Jl.toString(3).c_str());
            //printf("Cartesian J_pinv gains: %.3f %.3f %.3f\n", math::norm(Jl_pinv.getCol(0)), math::norm(Jl_pinv.getCol(1)), math::norm(Jl_pinv.getCol(2)));
            printf("n: %s\n", n.toString(3).c_str());
            printf("pressure: %.1f\n", c->getPressure());
            printf("dq_d: %s\n", qVel.toString(1).c_str());
#endif
        }

    }
    Vector qMin = CTRL_RAD2DEG * limb->getJointBoundMin();
    Vector qMax = CTRL_RAD2DEG * limb->getJointBoundMax();
    double jointSafetyMargin = 5.0;
    for(unsigned int i=0; i<N; i++)
    {
        if(out[i]>ref.dqMax[i])
            out[i] = ref.dqMax[i];
        else if(out[i]<-ref.dqMax[i])
            out[i] = -ref.dqMax[i];

        if(out[i]<0.0 && s.q[i] < qMin[i]+jointSafetyMargin){
            printf("Lower limit\n");
            out[i] = 0.0;
        }
        else if(out[i]>0.0 && s.q[i]>qMax[i]-jointSafetyMargin){
            printf("Upper limit\n");
            out[i] = 0.0;
        }

    }
    out *= s.activeJoints;                                // remove blocked joints

    feasibility = 1.0;
    return Status();
} 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status PressureControlLaw::getParameter(const string &key, Vector &v){
    if(key=="Kp"){
        v = getKp();
        return Status();
    }
    Status s = IControlLaw::getParameter(key, v);
    if(s) return s;
    return Status("Error while trying to set the parameter "+key+": unknown parameter name");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status PressureControlLaw::setKp(const Vector &_kp)
{ 
    if(_kp.size()!=kp.size())
        return Status("Error while setting k: wrong size");
    mutex.wait();
    kp = _kp; 
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status PressureControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kp:   reply.addString(getKp().toString(3).c_str()); break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s) break;
                switch(cmdId){
                    case set_kp:
                        if(v.size()==1)
                            s = setKp(Vector(3, v[0]));
                        else
                            s = setKp(v);
                        break;
                    default:
                        return IControlLaw::respond(cmdId, params, reply);
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}







//====================================
//
//		SafeReachRigidControlLaw
//
//====================================
SafeReachRigidControlLaw::SafeReachRigidControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, 
                const Vector &_kv, const Vector &_kf, const Vector &_kd, const yarp::sig::Vector &_krest, double _fThresh)
                : ForceFieldJointBoundControlLaw(_limb, _period, _kf, zeros(_kf.size()), zeros(_kf.size())), 
                kp(_kp), kv(_kv), kd(_kd), krest(_krest), fThreshMax(Vector(1,_fThresh)), fThreshMin(Vector(1,0.1*_fThresh))
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SafeReachRigidControlLaw::SafeReachRigidControlLaw(iDynLimb* _limb, int _period, const Vector &_kp, const Vector &_kv, 
        const Vector &_kf, const Vector &_kd, const yarp::sig::Vector &_krest, double _fThresh, const Vector &_kj, 
        const Vector &_activationThresholds, const Vector &_deactivationThresholds)
        : ForceFieldJointBoundControlLaw(_limb, _period, _kf, zeros(_kf.size()), zeros(_kf.size()), 
        _kj, _activationThresholds, _deactivationThresholds),
        kp(_kp), kv(_kv), kd(_kd), krest(_krest), fThreshMax(Vector(1,_fThresh)), fThreshMin(Vector(1,0.1*_fThresh))
{
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SafeReachRigidControlLaw::init()
{
    ForceFieldJointBoundControlLaw::init();
    name = "SafeReachRigidControlLaw";
    shortName = "cont";
    mutex.setName(name+"Mutex");
    ctrlMode = TORQUE_CTRL_MODE;
    int cmdSize = SafeReachRigidControlLawCommandSize - commandList.size();
    commandList.insert(commandList.end(), SafeReachRigidControlLawCommand_s, SafeReachRigidControlLawCommand_s+cmdSize);
    commandDesc.insert(commandDesc.end(), SafeReachRigidControlLawCommand_desc, SafeReachRigidControlLawCommand_desc+cmdSize);
    cltype = CL_RIGID;

    monitor = zeros(50);
    jb = zeros(N);
    w = zeros(1,6);
    R_r_c = eye(3);
    Jc = dJc = Jlink = dJlink = zeros(6, N);
    J = dJ = JNf = zeros(3, N);
    dJf = Jf = zeros(1, N);
    Jf_pinv = zeros(N, 1);
    JNf_pinv = zeros(N, 3);
    Nf = eye(N);

    w_d = z6 = zeros(6); 
    svJ = svJNf = x = dx = ctrlPoint_r = ctrlPoint2_r = z3 = zeros(3);
    J_pinv = zeros(N, 3);
    isInContact = false;

    forceRefGen = new minJerkTrajGen(zeros(6), period*1e-3, 2.0);   // reference force generator
    forceThrGen = new minJerkTrajGen(fThreshMax, period*1e-3, 3.0); // force threshold generator
    currentFThresh = fThreshMax(0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status SafeReachRigidControlLaw::reset(const CtrlRef &ref, const RobotStatus &s)
{
    Status ss = ForceFieldJointBoundControlLaw::reset(ref, s);
    if(!ss) return ss;
    referenceChanged(ref, s);   // reset minJerkTrajGen
    return ss;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SafeReachRigidControlLaw::referenceChanged(const CtrlRef &ref, const RobotStatus &s)
{
    
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SafeReachRigidControlLaw::computeJacobians(const CtrlRef &ref, const RobotStatus &s)
{
    dqRad = CTRL_DEG2RAD * s.dq;                // joint velocities in rad/sec
    limb->setAng(CTRL_DEG2RAD * s.q);           // should be already set, but just in case
    limb->setDAng(dqRad);
    
    Jlink.zero(); dJlink.zero(); J.zero(); dJ.zero();
    J.setSubmatrix(limb->GeoJacobian(ref.linkNumber).submatrix(0,2,0,N-1), 0, 0);
    dJ.setSubmatrix(limb->DJacobian(ref.linkNumber, dqRad).submatrix(0,2,0,N-1), 0, 0);
    Jlink.setSubmatrix(limb->GeoJacobian(ref.linkNumber2), 0, 0);
    dJlink.setSubmatrix(limb->DJacobian(ref.linkNumber2, dqRad), 0, 0);
    //set to zero the cols of J corresponding to blocked joints
    for(unsigned int i=0; i<N; i++)
        if(s.activeJoints[i]==0.0){
            J.setCol(i, z3); dJ.setCol(i, z3);
            Jlink.setCol(i, z6); dJlink.setCol(i, z6);
        }

    // assume the control point is always [0 0 0]
    rototranslate(limb->getH(ref.linkNumber), ref.ctrlPoint, x);
    dx = J*dqRad;
    
    rotate(limb->getH(ref.linkNumber2), ref.ctrlPoint2, ctrlPoint2_r);    // contact point w.r.t. link in root ref frame
    Jc = contactPointJacobian(Jlink, ctrlPoint2_r);
    if(isInContact && (cltype==CL_RIGID || cltype==CL_ORTHO))
    {
        printf("CONTACT\n");
        dJc = contactPointJacobian(dJlink, ctrlPoint2_r);
        w.setSubrow(versor(s.wrench.subVector(0,2)),0,0);
        Jf  = w*Jc;
        dJf = w*dJc;
        //pinvDamped(Jf, Jf_pinv, ref.pinvDamp);
        //pinv(Jf, Jf_pinv, ref.pinvDamp);
        pinv(Jf, Jf_pinv, 1e-5);
    }
    else
    {
        dJc.zero();
        Jf.zero(); dJf.zero();
        Jf_pinv.zero();
    }
    Nf = eye(N) - Jf_pinv*Jf;
    JNf = J*Nf;
    pinvDamped(JNf, JNf_pinv, svJNf, ref.pinvDamp);
    Na = Nf*(eye(N)-pinv(JNf, 1e-5)*JNf);   // don't use damping when computing nullspace projector
    // just for debug
    pinvDamped(J, J_pinv, svJ, ref.pinvDamp);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status SafeReachRigidControlLaw::abstractCompute(const CtrlRef &ref, const RobotStatus &s, Vector &out, double &feasibility)
{
    isSkinInContact = s.contactPressure>=5.0;
    isInContact     = isSkinInContact || norm(s.wrench)>currentFThresh;

    forceThrGen->computeNextValues(isSkinInContact?fThreshMin:fThreshMax);
    currentFThresh = forceThrGen->getPos()(0);

    forceRefGen->computeNextValues(ref.wrenchRef);
    if(!isInContact) forceRefGen->init(s.wrench);
    w_d = forceRefGen->getPos();

    currentRef = ref;
    computeJacobians(ref, s);
    computeJntBoundCtrl(s, jb);     // joint bound ctrl

    // primary task
    ddx_d   = ref.ddx + kv*(ref.dx - dx) + kp*(ref.x-x);
    ddq_f   = -1.0*(Jf_pinv*(dJf*dqRad));
    y       = ddx_d-dJ*dqRad;
    ddq_d   = ddq_f + JNf_pinv*(y-J*ddq_f);

    // secondary task
    ddq_0   = Na*CTRL_DEG2RAD*(krest*(ref.q-s.q) + kd*(ref.dq-s.dq) + ref.ddq);

    // inverse dynamics
    limb->setD2Ang(ddq_d+ddq_0);
    limb->computeNewtonEuler(s.w0, s.dw0, s.d2p0, z3, z3);
    out     = limb->getTorques();
    
    if(cltype==CL_RIGID)
    {
        if(isInContact)
            out -= w_d*Jc;          // force feedforward
    }
    else if(cltype==CL_ORTHO)
    {
        if(isInContact)
            out *= Nf;              // project torques in contact force nullspace
    }
    
    mergeJntBoundCtrl(jb, out);     // merge joint bound ctrl torque with task ctrl torque
    out     *= s.activeJoints;      // remove blocked joints

    // task feasibility is the percentage of task space acceleration that can be achieved
    // i.e. 1 - (||ddx_d - ddx|| / ||ddx_d||)
    double norm_y = norm(y);
    feasibility = norm_y>1e-2 ? (1.0 -  norm(y-J*ddq_d)/norm_y) : 1.0;

    sendMonitorData();

#define _DEBUG
#ifdef _DEBUG
    printf("current f thresh: %.1f\n", currentFThresh);
    printf("current fd:   %.1f\n", norm(w_d.subVector(0,2)));
    
    printf("ddx_d: %s\n", ddx_d.toString(2).c_str());
    printf("ddx:   %s\n", (J*ddq_d+dJ*dqRad).toString(2).c_str());
    //printf("ddx_f: %s (should be about zero)\n", (Jf*ddq_d+dJf*dqRad).toString(2).c_str());
    printf("Jf:      %s\n", Jf.toString(2).c_str());
    printf("Jfpinv:  %s\n", Jf_pinv.transposed().toString(2).c_str());
    printf("SV(J):   %s\n", svJ.toString(3).c_str());
    printf("SV(JNf): %s\n", svJNf.toString(3).c_str());
    printf("Cart JNf_pinv gains: %.3f %.3f %.3f\n", math::norm(JNf_pinv.getCol(0)), math::norm(JNf_pinv.getCol(1)), math::norm(JNf_pinv.getCol(2)));
    //printf("Jf*JNfpinv: %s (should be zero)\n", (Jf*JNf_pinv).toString(2).c_str());
    //printf("ddq_f: %s\n", ddq_f.toString(2).c_str());
    printf("ddq_d: %s\n", ddq_d.subVector(3,6).toString(2).c_str());
    printf("ddq_0: %s\n", ddq_0.subVector(3,6).toString(2).c_str());
    Vector tau_acc = (out - s.activeJoints*limb->computeCcGravityTorques(s.d2p0));
    printf("tau_acc: %s\n", tau_acc.subVector(3,6).toString(3).c_str());
    printf("\n");
#endif

    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SafeReachRigidControlLaw::sendMonitorData()
{
    // MONITOR DATA
    monitor.zero();
    unsigned int index = 0;
    monitor.resize(9);
    // 0-2 ddx_d
    monitor.setSubvector(index, ddx_d); index+=3;
    // 3-5 ddx
    monitor.setSubvector(index, J*ddq_d + dJ*dqRad); index+=3;
    // 6-8 ddx without force nullspace projection
    monitor.setSubvector(index, J*J_pinv*y + dJ*dqRad); index+=3;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status SafeReachRigidControlLaw::setParameter(const string &key, const Vector& value){
    if(key == "Kd")
        return setKd(value);
    Status s = ForceFieldJointBoundControlLaw::setParameter(key, value);
    if(s)  return s;
    if(key == "Kf")
        return setKf(value);
    return s && Status("Error while trying to set the parameter "+key+": unknown parameter name");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status SafeReachRigidControlLaw::getParameter(const string &key, Vector &v){
    if(key=="Kd"){
        v = getKd();
        return Status();
    }
    Status s = ForceFieldJointBoundControlLaw::getParameter(key, v);
    if(s) return s;
    if(key=="Kf"){
        v = getKf();
        return Status();
    }
    return Status("Error while trying to set the parameter "+key+": unknown parameter name");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status SafeReachRigidControlLaw::respond(unsigned int cmdId, Bottle &params, Bottle& reply){
    Status s;   // true if everything goes fine
    switch(cmdId){
        case get_kp:        reply.addString(getKp().toString(3).c_str());       break;
        case get_kd:        reply.addString(getKd().toString(3).c_str());       break;
        case get_kf:        reply.addString(getKf().toString(3).c_str());       break;
        case get_kv:        reply.addString(getKv().toString(3).c_str());       break;
        case get_krest:     reply.addString(getKrest().toString(3).c_str());    break;
        case get_clType:    reply.addInt(cltype);                               break;
        default:
            {
                Vector v;
                s = bottleToVector(params, v);
                if(!s || v.size()==0) 
                    break;
                switch(cmdId){
                    case set_kp:    s = setKp(v.size()==1 ? Vector(3, v[0]) : v);   break;
                    case set_kf:    s = setKf(v.size()==1 ? Vector(pidN, v[0]) : v);break;
                    // reimplement "set kd" because kd dimension isn't pidN, but N
                    case set_kd:    s = setKd(v.size()==1 ? Vector(N, v[0]) : v);   break;
                    case set_kv:    s = setKv(v.size()==1 ? Vector(3, v[0]) : v);   break;
                    case set_krest: s = setKrest(v.size()==1 ? Vector(N,v[0]) : v); break;
                    case set_clType:s = setClType((int)v[0]);                       break;
                    default:        return JointBoundControlLaw::respond(cmdId, params, reply);
                }
            }
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command executed.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return Status();    // return true because the command has been recognized
}

}

} // end namespace
