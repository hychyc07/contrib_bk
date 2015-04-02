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

#include "iCub/skinForceControl/controlPlanner.h"
#include "iCub/skinForceControl/util.h"
#include "iCub/skinForceControl/utilRobotics.h"
#include "iCub/skinDynLib/common.h"

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <iostream>

using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;
using namespace iCub::iKin;


controlPlanner::controlPlanner(string _moduleName, IController* _controller, int _period, VerbosityLevel _verbose) throw()
: RateThread(_period), controller(_controller), verbose(_verbose)
{
    //---------------------PORTS-------------------------//	
    string slash = "/";
    port_monitor        = new BufferedPort<Vector>;
	if(!port_monitor->open((slash+_moduleName+"/monitor:o").c_str()))
        thread_status.addErrMsg("It was not possible to open the monitor:o port");
    
	//----------- INIT VARIABLES----------------//
    action			    = NO_ACTION;
    pastAction			= NO_ACTION;
    actionChanged       = true;
    ctrlRef             = controller->getCtrlRef();
    robot               = controller->getRobotStatus();
    feasibility         = controller->getFeasibility();
    isRefStable         = true;

    commandList.assign(IPlannerCommand_s, IPlannerCommand_s+IPlannerCommandSize);
    commandDesc.assign(IPlannerCommand_desc, IPlannerCommand_desc+IPlannerCommandSize);
    actionSem.setName("ctrlPlannerMutex");

    //----------- INIT ACTIONS----------------//
    actions[NO_ACTION]      = new NoAction(controller, (int)getRate());
    actions[LINE_SCRUB]     = new LineScrub(controller, (int)getRate(), SCRUB_FORCE, SCRUB_LINE_PATH);
    actions[ADAP_LSCRUB]    = new AdaptiveLineScrub(controller, (int)getRate(), SCRUB_FORCE, SCRUB_LINE_PATH, DEFAULT_ETA);
    actions[CIRCLE]         = new Circle(controller, (int)getRate(), CIRCLE_RAY, CIRCLE_VEL);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool controlPlanner::threadInit(){ 
    DSemaphore::registerThread("CtrlPlanner");
    return true; 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::threadRelease(){
    sendMsg("Control planner release.\n");
    if(port_monitor){ port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::run(){
    // check everything is working fine
    sanityCheck();
    if( !thread_status )
        return;
    if( feasibility < FEASIBILITY_THR ){
        // if the task is not feasible stop the controller
        setAction(NO_ACTION);
        sendMsg("Stopping the planner. Task unfeasible ("+toString(feasibility)+"<"+toString(FEASIBILITY_THR)+").", MSG_WARNING);
        feasibility = 1.0;  // reset the feasibility
    }

    // read robot status
    updateRobotStatus();

    // CONTROL LOOP
    Action currentAction;
    actionSem.wait();
    {
        currentAction = action;
        if(actionChanged){
            // if the action has changed, initialize the new action
            sendMsg("Action just changed to "+Action_desc[currentAction], MSG_INFO);
            releaseAction(pastAction);
            initAction(currentAction);
            actionChanged = false;
        }
    }
    actionSem.post();

    if(executeAction(currentAction) == DONE)
        sendMsg("Action "+Action_desc[currentAction]+"finished");

    // send monitoring data
    sendMonitorData(currentAction);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::updateRobotStatus() throw(){
    //ctrlRef             = controller->getCtrlRef();
    //robot               = controller->getRobotStatus();
    //feasibility         = controller->getFeasibility();
    //xd                  = controller->getXd();
    //fd                  = controller->getFd();

    //if(ctrlRef.x.size()!=7) printf("ctrlRef.x.size()!=7\n");
    //if(xd.size()!=7) printf("xd.size()!=7\n");
    //if(ctrlRef.wrench.size()!=6) printf("ctrlRef.wrench.size()!=6\n");
    //if(fd.size()!=6) printf("fd.size()!=6\n");

    //if(norm((ctrlRef.x-xd).subVector(0,2))<XD_STAB_THR && norm((ctrlRef.wrench-fd).subVector(0,2))<FD_STAB_THR)
    //    isRefStable = true;
    //else
    //    isRefStable = false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::initAction(const Action &cp) throw(){
    Status s;
    if(!(s=actions[cp]->initAction()))
        sendMsg(s.toString(), MSG_WARNING);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ActionStatus controlPlanner::executeAction(const Action &cp) throw(){
    return actions[cp]->executeAction();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::releaseAction(const Action &cp) throw(){
    Status s;
    if(!(s=actions[cp]->releaseAction()))
        sendMsg(s.toString(), MSG_WARNING);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::sanityCheck() throw(){
    if(controller==NULL){
        sendMsg("Reference to controller lost.", MSG_ERROR);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::sendMonitorData(const Action &cm) throw(){
    // if nobody is reading do not write
    if(port_monitor->getOutputCount()==0)
        return;
    /*int index = 0;
    monitorData.resize(1);
    port_monitor->prepare() = monitorData;
    port_monitor->write();*/
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlPlanner::sendMsg(const string &msg, const MsgType &type) throw(){
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
