/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

#define RET_INVALID     -1
YARP_DECLARE_DEVICES(icubmod)


bool respectWSLimit(yarp::sig::Vector& pose)
{
    if (pose[0] < -0.6 || pose[0] > -0.1 || pose[1] < -0.5 || pose[1] >0.5 || pose[2] < -0.05 || pose [2] >0.40)
        return false;
    else return true;
}

DmpExecutorThread::DmpExecutorThread(int period, yarp::os::Property &opt): RateThread(period), optPD(opt)
{
    cartCtrl=NULL;
    currentDMP=NULL;
    name=opt.find("threadName").asString().c_str();
}

bool DmpExecutorThread::threadInit()
{
    bool configOk=cartCtrlPD.open(optPD);

    if (cartCtrlPD.isValid()) 
    {
        configOk=cartCtrlPD.view(cartCtrl) && configOk;
    }
    else configOk=false;
    
    outputPort.open(("/" + name + "/pose:o").c_str());
    
    configOk= configOk && cartCtrl->storeContext(&initial_context);
    return configOk;
}

void DmpExecutorThread::threadRelease()
{
    if (cartCtrl!=NULL) 
    {   
        cartCtrl->stopControl();
        cartCtrl->restoreContext(initial_context);
    }
     
    cartCtrlPD.close();  
    outputPort.close();
}

void DmpExecutorThread::setDMP(dmp::DMP* newDMP)
{
    mutex.wait();
    if (currentDMP!=NULL)
        delete currentDMP;
    if (newDMP == NULL)
    {
        std::cout << "new dmp is null\n";
        mutex.post();
        return;
    }
    else
        currentDMP=newDMP->clone();
    
    Vector x0, o0, v0, a0;
    cartCtrl->getPose(x0,o0);
    cartCtrl->getTaskVelocities(v0,a0);
    
    yarp::sig::Vector pos0=yarp::math::cat(x0, o0);
    yarp::sig::Vector vel0=yarp::math::cat(v0, a0);
    currentDMP->set_positions(pos0);
    currentDMP->set_velocities(vel0);
    
    currentDMP->resetState();
    
    std::cout << " set DMP: " << *currentDMP << std::endl;
    mutex.post();
    //resume();
}

void DmpExecutorThread::run()
{
  //  std::cout << name << ": running" <<std::endl;
    mutex.wait();
    if (currentDMP ==NULL)
    {
     //   std::cout << name <<": dmp null" <<std::endl;
      //  suspend();
        mutex.post();
        return;
    }
        
    if (!currentDMP->isValid())
    {
        std::cout << name << ":dmp not valid" <<std::endl;
      //  suspend();
        mutex.post();
        return;
    }

    if (!(currentDMP->integrate(getRate()/1000)))
    {
        mutex.post();
        std::cout << name << ":integration failed" <<std::endl;
        suspend();
        
        return;
    }
    
    Vector xodotd=currentDMP->get_velocities();
    Vector xod=currentDMP->get_positions();

    mutex.post();
    //std::cout << name << ": moving hand X to " << 
    std::cout <<xod.toString()<< std::endl;
    if (!respectWSLimit(xod))
    {
      //  std::cout << "!! out of WS limits!!" <<std::endl;
        return;
    }

    cartCtrl->setTaskVelocities(xodotd.subVector(0,2), xodotd.subVector(3,6));
    cartCtrl->goToPose (xod.subVector(0,2), xod.subVector(3,6));
    // cartCtrl->goToPosition (xod.subVector(0,2));
                   
    outputPort.prepare()=xod;
    outputPort.write();
}

/**********************************************************/
DmpExecutor::DmpExecutor()
{
    YARP_REGISTER_DEVICES(icubmod)
    currentDMP=NULL;
   
    //defaultHand=iCub::RIGHT;
    defaultHand=iCub::INDIFF;
    currentHand=iCub::RIGHT;
    currentDMP=NULL;
    
    execThreadLeft=NULL;
    execThreadRight=NULL;
    kinTeachLeft=NULL;
    kinTeachRight=NULL;
    teachingAvailable=false;
    
}
/**********************************************************/
bool DmpExecutor::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}
/**********************************************************/
bool DmpExecutor::run()
{

    if (execThreadLeft->isSuspended())
        execThreadLeft->resume();
    if (execThreadRight->isSuspended())
        execThreadLeft->resume();
  return true;
}
/**********************************************************/
bool DmpExecutor::is_running()
{
    return ((execThreadLeft->isRunning() && ! execThreadLeft->isSuspended()) || (execThreadRight->isRunning() && ! execThreadRight->isSuspended()));
}
/**********************************************************/
bool DmpExecutor::stop()
{
    execThreadLeft->suspend();
    execThreadRight->suspend();
    if (teachingAvailable)
    {
        if (kinTeachLeft)
        {
            Bottle action;
            if (currentHand==iCub::LEFT && kinTeachLeft->stopRecodingAction(action))
                opcPort.setAction(actionName, &action);
        }
        if (kinTeachRight)
        {
            Bottle action;
            if (currentHand==iCub::RIGHT && kinTeachRight->stopRecodingAction(action))
                opcPort.setAction(actionName, &action);
        }
    }
    return true;
}
/**********************************************************/
bool DmpExecutor::execute_OPC(const int32_t id)
{
    
     std::cout << "asking opc" <<std::endl;

     mutex.wait();
     if (currentDMP)
         delete currentDMP;
    currentDMP=opcPort.get_information_for(id);
    mutex.post();
    if (currentDMP ==NULL || !currentDMP->isValid())
    {
        std::cout << "OPC returned an invalid DMP" << std::endl; fflush(stdout);
        return false;
    }

    if (defaultHand==iCub::INDIFF)
    {
        currentHand= ((currentDMP->get_attractor())(1)<0) ? iCub::LEFT : iCub::RIGHT; // checking for position of target along Y coord of root frame
    }
        
    bool ok=true;
    switch (currentHand)
    { // TODO: ADD CHECKS ON CARTESIAN INTERFACE SUCCESS
        case iCub::LEFT:
            std::cout << "LEFT";
            execThreadLeft->setDMP(currentDMP);
            execThreadLeft->resume();
            if (!execThreadLeft->isRunning())
                std::cout << " not running!";

            break;
        case iCub::RIGHT:
            std::cout << "RIGHT";
            execThreadRight->setDMP(currentDMP);
            execThreadRight->resume();
            if (!execThreadRight->isRunning())
                std::cout << " not running!";             
            break;
            
        default:
            ok=false;
    }

    return ok;
}
/**********************************************************/    
bool  DmpExecutor::set_hand(const iCub::Hand newHand)
{
    defaultHand=newHand;
    if (!is_running() && newHand != iCub::INDIFF)
        currentHand=newHand;
    return !is_running();
    
}
/**********************************************************/
iCub::Hand  DmpExecutor::get_hand()
{
    return defaultHand; //or current?? need to decide..
}
/**********************************************************/
bool DmpExecutor::waitMotionDone(const double period, const double timeout)
{
    /*bool done=false;
    double t0=Time::now();

    while (!done)
    {
        Time::delay(period);

        done=!is_running();
        if (!done || ((timeout>0.0) && ((Time::now()-t0)>timeout)))
            return false;
    }*/

    return true;
}
/**********************************************************/
 bool DmpExecutor::teach_start(const std::string& actionName, const iCub::Hand handToUse)
 {
     if (!teachingAvailable)
         return false;
     mutex.wait();
     currentHand=handToUse;
     this->actionName=actionName;
     mutex.post();
     Time::delay(5.0);
     if(handToUse==iCub::LEFT)
        return kinTeachLeft->startRecordingAction(actionName);
     
     if(handToUse==iCub::RIGHT)
         return kinTeachRight->startRecordingAction(actionName);
     
     return false;
 }

/**********************************************************/
bool DmpExecutor::configure(ResourceFinder &rf)
{
    bool configOk=true;
    setName(rf.find("name").asString().c_str());
    string robot=rf.check("robot", yarp::os::Value("icub")).asString().c_str();
    std::cout << "robot " << robot <<std::endl;
    
    period=rf.check("period", Value(0.1)).asDouble();
    std::cout << "period " << period <<std::endl;
    string slash="/";
 
    thriftPort.open((slash+string(getName().c_str())+"/thrift:rpc").c_str());   
    configOk = attach(thriftPort) && configOk;
    
    opcPort.open((slash+string(getName().c_str())+"/opc:rpc").c_str());
    

    Property optionR;
    optionR.put("device","cartesiancontrollerclient");
    optionR.put("remote",(slash + robot +"/cartesianController/right_arm").c_str());
    optionR.put("local",(slash + getName().c_str() +"/client/right_arm").c_str());
    optionR.put("threadName", "rightArmDMPexecutor");
    
    execThreadRight= new DmpExecutorThread(int(period*1000), optionR);

    configOk= execThreadRight->start() && configOk;

    Property optionL;
    optionL.put("device","cartesiancontrollerclient");
    optionL.put("remote",(slash + robot +"/cartesianController/left_arm").c_str());
    optionL.put("local",(slash + getName().c_str() +"/client/left_arm").c_str());
    optionL.put("threadName", "leftArmDMPexecutor");
    
    
    execThreadLeft= new DmpExecutorThread(int(period*1000), optionL);
    
    configOk= execThreadLeft->start() && configOk;

    if (rf.check("teaching"))
    {
        yarp::os::Property kinProp;
        kinProp.put("robot", robot.c_str());
        kinProp.put("threadName", "kinTeachLeft");
        kinProp.put("part", "left_arm");
        kinProp.put("impedence_arm_stiffness", rf.find("impedence_arm_stiffness"));  //better to read from config file
        kinProp.put("impedence_arm_damping", rf.find("impedence_arm_damping"));
        kinProp.put("subsampling_rate", rf.find("subsampling_rate"));
        kinTeachLeft= new KinTeachingThread(int (period*1000), kinProp);
        
        std::cout << "created kin thread\n";
        configOk= kinTeachLeft->start() && configOk;
        std::cout << "started kin thread\n";
        kinTeachLeft->suspend();
        
        std::cout << "suspended kin thread\n";
        
        kinProp.put("threadName", "kinTeachRight");
        kinProp.put("part", "right_arm");
        kinTeachRight=new KinTeachingThread(int(period*1000), kinProp);   
        
        std::cout << "created kin thread\n";
        configOk= kinTeachRight->start() && configOk;
        kinTeachRight->suspend();
        teachingAvailable=true;
    }
    
    
    return configOk;
}

/**********************************************************/
bool DmpExecutor::interruptModule()
{

    opcPort.interrupt();
    thriftPort.interrupt();

    return true;
}
/**********************************************************/
bool DmpExecutor::close()
{

    if (kinTeachLeft)
    {
        kinTeachLeft->stop();
        delete kinTeachLeft;
    }
    
    if (kinTeachRight)
    {
        kinTeachRight->stop();
        delete kinTeachRight;
    }
    

    if (execThreadLeft)
    {
        execThreadLeft->stop();
        delete execThreadLeft;
    }
    
    if (execThreadRight)
    {
        execThreadRight->stop();
        delete execThreadRight;
    }
    
    if (currentDMP)
        delete currentDMP;
    
    opcPort.close();
    thriftPort.close();

    return true;
}
/**********************************************************/
bool DmpExecutor::updateModule()
{
    if (isStopping())
        return false;

    return true;
}
/**********************************************************/
double DmpExecutor::getPeriod()
{
    return period;
}

void DmpExecutor::quit()
{
    stopModule();
}
