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

#include <iCub/skinForceControl/actions.h>
#include <iCub/skinForceControl/util.h>
#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <sstream>

using namespace std;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::iDyn;
using namespace iCub::skinForceControl;

namespace iCub
{

namespace skinForceControl
{

//====================================
//
//		IAction
//
//====================================
IAction::IAction(){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
IAction::IAction(IController* _controller, int _period): controller(_controller), period(_period){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus IAction::executeAction(){
    if(status==DONE)
        return status;
    // acquire the mutex to be sure that the parameters are not modified while the 
    // action execution is ongoing
    mutex.wait();
    ActionStatus res = abstractExecute();
    mutex.post();
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IAction::respond(const Bottle& command, Bottle& reply){
    Bottle params;
    unsigned int cmdId=0;
    if( ! identifyCommand(command, commandList, cmdId, params))
        return Status("Command unknown");
    return respond(cmdId, params, reply);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IAction::respond(unsigned int &cmdId, Bottle &params, Bottle& reply){
    Status s;
    switch(cmdId){
        case get_status:    reply.addString(ActionStatus_s[getStatus()].c_str()); break;
        case resetCmd:      reset(); break;
        default:            s.addErrMsg("Command not managed");
    }
    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command received.").c_str());
    else if(!s)
        reply.addString(s.toString());
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status IAction::reset(){
    status = ONGOING;
    return Status();
}


//====================================
//
//		NoAction
//
//====================================
NoAction::NoAction(IController* _controller, int _period)
:IAction(_controller, _period){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void NoAction::init(){
    name = "NoAction";
    shortName = "noac";
    mutex.setName(name+"Mutex");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status NoAction::initAction(){
    controller->setCtrlLaw(NO_CONTROL);
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus NoAction::abstractExecute(){
    return ONGOING;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status NoAction::releaseAction(){ return Status(); }


//====================================
//
//		LineScrub
//
//====================================
LineScrub::LineScrub(IController* _controller, int _period)
:IAction(_controller, _period), scrubForce(zeros(3)), scrubPath(zeros(3)), scrubVel(zeros(3)){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LineScrub::LineScrub(IController* _controller, int _period, const Vector& _scrubForce, const Vector& _scrubPath)
:IAction(_controller, _period), scrubForce(_scrubForce), scrubPath(_scrubPath), scrubVel(zeros(3)){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::initAction(){
    currentPhase = scrub_back;
    controller->setTrajTime(DEFAULT_TRAJ_TIME_XD);
    Status s = controller->setCtrlPnt(controller->getX());
    xd = controller->getCtrlPoint().subVector(0,2);
    xd_link = xd_link.subVector(1,3);
    s &= controller->setFd(cat(scrubForce, zeros(3)));
    Time::delay(1.5*controller->getTrajTime());

    if(s){
        if(dynOn)
            controller->setCtrlLaw(DYN_PARAL_CTRL);
        else
            controller->setCtrlLaw(PARAL_CTRL);
    }
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus LineScrub::abstractExecute(){
    // check whether the references (force and position) are stable
    CtrlRef ref = controller->getCtrlRef();
    Vector fd = controller->getFd();
    Vector f = controller->getF();
    bool isRefStableX = norm(scrubPath)==0?true:norm(ref.x.subVector(0,2)-xd)<0.01*norm(scrubPath);
    bool isRefStableF = norm(scrubForce)==0?true:norm(ref.wrench-fd)<0.2*norm(fd);
    bool isRefStable = isRefStableX && isRefStableF;
        
    // turn the desired force towards the direction of the measured force
    double norm_f = norm(f);
    if(norm_f > norm(scrubForce)/2.0){
        fd = alphaFdir*fd + (1.0-alphaFdir)*(f*norm(fd)/norm_f);
    }else{
    	fd = alphaFdir*fd + (1.0-alphaFdir)*cat(scrubForce, zeros(3));
    }
	controller->setFd(fd);

	
    // if the references are stable set the new desired position xd
    if(isRefStable){
        if(currentPhase==scrub_back){
            currentPhase = scrub_forth;
            xd += scrubPath;
            xd_link += scrubPath;
        }
        else{
            currentPhase = scrub_back;
            xd -= scrubPath;
            xd_link -= scrubPath;
        }
        controller->setCtrlPnt(xd);
    }
    return ONGOING;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::releaseAction(){
    controller->setCtrlLaw(NO_CONTROL);
    Status s = controller->setCtrlPnt(controller->getX());
    s &= controller->setFd(zeros(6));
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::respond(const Bottle& command, Bottle& reply){
    unsigned int cmdId;
    Bottle params;
	if(!identifyCommand(command, commandList, cmdId, params))
		return Status("Command unknown");
    return respond(cmdId, params, reply);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::respond(unsigned int &cmdId, Bottle &params, Bottle& reply){
    Status s;
    switch(cmdId){
        case get_scrub_force:   reply.addString(scrubForce.toString(1).c_str()); break;
        case get_scrub_path:    reply.addString(scrubPath.toString(3).c_str()); break;
        case get_scrub_vel:     reply.addString(scrubVel.toString(3).c_str()); break;
        case get_alphaFdir:     reply.addDouble(alphaFdir); break;
        case set_scrub_force:   s = setScrubForce(params); break;
        case set_scrub_path:    s = setScrubPath(params); break;
        case set_scrub_vel:     s = setScrubVel(params); break;
        case set_alphaFdir:     s = setAlphaFdir(params); break;
        case dynamics_on:       dynOn = true;   controller->setCtrlLaw(DYN_PARAL_CTRL); break;
        case dynamics_off:      dynOn = false;  controller->setCtrlLaw(PARAL_CTRL);     break;
        default:                s = IAction::respond(cmdId, params, reply);
    }

    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command received.").c_str());
    else
        reply.addString(s.toString());
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::setScrubForce(const Bottle& params){
    Vector v;
    Status s;
    if(!(s=bottleToVector(params, v)))
        return s;
    if(v.size() != scrubForce.size())
        return Status("Wrong size of scrub force");
    scrubForce = v;
    mutex.wait();
    s = controller->setFd(v);
    mutex.post();
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::setScrubVel(const Bottle& params){
    Vector v;
    Status s;
    if(!(s=bottleToVector(params, v)))
        return s;
    if(v.size() != scrubVel.size())
        return Status("Wrong size of scrub vel");
    mutex.wait();
    scrubVel = v;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::setScrubPath(const Bottle& params){
    Vector v;
    Status s;
    if(!(s=bottleToVector(params, v)))
        return s;
    if(v.size() != scrubPath.size())
        return Status("Wrong size of scrub path");
    mutex.wait();
    scrubPath = v;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status LineScrub::setAlphaFdir(const Bottle& params){
    if(params.size()!=1)
        return Status("Parameter missing or too many parameters specified");
    if(!params.get(0).isDouble())
        return Status("The specified value is not a double");
    double a = params.get(0).asDouble();
    if(a<0.0 || a>1.0)
        return Status("Parameter is not in [0, 1]");
    mutex.wait();
    alphaFdir = a;
    mutex.post();
    return Status();
}


//====================================
//
//		AdaptiveLineScrub
//
//====================================
AdaptiveLineScrub::AdaptiveLineScrub(IController* _controller, int _period)
:LineScrub(_controller, _period), eta(0.0){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
AdaptiveLineScrub::AdaptiveLineScrub(IController* _controller, int _period, const Vector& _scrubForce, const Vector& _scrubPath, const double& _eta)
:LineScrub(_controller, _period, _scrubForce, _scrubPath), eta(_eta){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status AdaptiveLineScrub::initAction(){
    return LineScrub::initAction();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus AdaptiveLineScrub::abstractExecute(){
    Vector f = controller->getF().subVector(0,2);
    xd += eta*(f-scrubForce);
    controller->setCtrlPnt(xd);
    return LineScrub::abstractExecute();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status AdaptiveLineScrub::releaseAction(){
    return LineScrub::releaseAction();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status AdaptiveLineScrub::respond(const Bottle& command, Bottle& reply){
    return LineScrub::respond(command, reply);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status AdaptiveLineScrub::respond(unsigned int &cmdId, Bottle &params, Bottle& reply){
    Status s;
    switch(cmdId){
        case get_eta:   reply.addDouble(eta); break;
        case set_eta:   
            if(params.size()<1 || !(params.get(0).isDouble() || params.get(0).isInt()))
                s.addErrMsg("Error: you have to specify a numeric value for eta");
            else
                s = setEta(params.get(0).asDouble());
            break;
        default:                
            s = LineScrub::respond(cmdId, params, reply);
    }

    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command received.").c_str());
    else
        reply.addString(s.toString());
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status AdaptiveLineScrub::setEta(const double& _eta){
    safeSet(eta, _eta, mutex);
    return Status();
}


//====================================
//
//		Circle
//
//====================================
Circle::Circle(IController* _controller, int _period)
:IAction(_controller, _period){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Circle::Circle(IController* _controller, int _period, double _circleRay, double _pathVel)
:IAction(_controller, _period), circleRay(_circleRay), pathVel(_pathVel){
    init();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::initAction(){
    currentAngle = 0.0;
    center = controller->getX();
    controller->setTrajTime(DEFAULT_TRAJ_TIME_XD);
    Status s = controller->setXd(center + cat(circleRay, 0.0, 0.0));
    s &= controller->setCtrlPnt(9, zeros(3));
    s &= controller->setFd(zeros(6));
    if(s){
        if(dynOn)
            controller->setCtrlLaw(SAFE_REACH_RIGID_CTRL);
        else
            controller->setCtrlLaw(PARAL_CTRL);
    }
    Time::delay(1.5*DEFAULT_TRAJ_TIME_XD);   // wait for end-effector to move to starting position
    controller->setTrajTime(0.5);
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus Circle::abstractExecute(){
    // increment the current angle
    double d_theta = pathVel*(period/1000.0)/circleRay;
    currentAngle += d_theta;
    // compute and set the new desired position
    double xd = center[0] + circleRay*cos(currentAngle);
    double yd = center[1] + circleRay*sin(currentAngle);
    //printf("xd: %.3f\tyd: %.3f\n", xd, yd);
    controller->setXd(cat(xd, yd, center[2]));

    return ONGOING;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::releaseAction(){
    controller->setCtrlLaw(NO_CONTROL);
    controller->setTrajTime(DEFAULT_TRAJ_TIME_XD);
    Status s = controller->setXd(controller->getX());
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::respond(const Bottle& command, Bottle& reply){
    unsigned int cmdId;
    Bottle params;
	if(!identifyCommand(command, commandList, cmdId, params))
		return Status("Command unknown");
    return respond(cmdId, params, reply);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::respond(unsigned int &cmdId, Bottle &params, Bottle& reply){
    Status s;
    switch(cmdId){
        case get_circle_ray:    reply.addDouble(circleRay); break;
        case get_circle_vel:    reply.addDouble(pathVel); break;
        case set_circle_ray:    s = setCircleRay(params); break;
        case set_circle_vel:    s = setCircleVel(params); break;
        case dynamics_on:       dynOn = true;   controller->setCtrlLaw(DYN_PARAL_CTRL); break;
        case dynamics_off:      dynOn = false;  controller->setCtrlLaw(PARAL_CTRL);     break;
        default:                s = IAction::respond(cmdId, params, reply);
    }

    if(s && reply.toString()=="")
        reply.addString( (commandList[cmdId]+" command received.").c_str());
    else
        reply.addString(s.toString());
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::setCircleRay(const Bottle& params){
    if(params.size()!=1)
        return Status("Parameter missing or too many parameters specified");
    if(!params.get(0).isDouble())
        return Status("The specified value is not a double");
    double a = params.get(0).asDouble();
    if(a<0.0)
        return Status("Parameter is not positive");
    mutex.wait();
    circleRay = a;
    mutex.post();
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status Circle::setCircleVel(const Bottle& params){
    if(params.size()!=1)
        return Status("Parameter missing or too many parameters specified");
    if(!params.get(0).isDouble())
        return Status("The specified value is not a double");
    double a = params.get(0).asDouble();
    if(a<0.0)
        return Status("Parameter is not positive");
    mutex.wait();
    pathVel = a;
    mutex.post();
    return Status();
}

}

} // end namespace
