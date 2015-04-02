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

#ifndef CTRL_PLANNER
#define CTRL_PLANNER

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/math.h>
#include "iCub/skinForceControl/controlConstants.h"
//#include "iCub/skinForceControl/controlThread.h"
#include "iCub/skinForceControl/actions.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{

namespace skinForceControl
{

const std::string IPlannerCommand_s[]  = {
    "get action", "stop action", "lscrub action", "alscrub action", "circle action" };

// the order in ControlPlannerCommand_desc must correspond to the order in ControlPlannerCommand_s
const std::string IPlannerCommand_desc[]  = {
    "get the current action",
    "stop the current action",
    "start the line scrub action",
    "start the adaptive line scrub action",
    "start the circle action" };

class controlPlanner: public yarp::os::RateThread
{
private:
    std::vector<std::string>    commandList;        // list of commands accepted by the planner
    std::vector<std::string>    commandDesc;        // description of commands accepted by the planner

    VerbosityLevel          verbose;
    Status                  thread_status;  // the status of the thread (OK, ERROR)
                                            // as soon as the status is ERROR, the module calls stop() on this thread

    BufferedPort<Vector>    *port_monitor;  // output port sending data for monitoring the controller
    Vector                  monitorData;    // vector sent on the monitor port
	
    IController*            controller;         // thread controlling the robot
    Action                  action;             // the current action executed
    Action                  pastAction;         // the action executed before the current action
    bool                    actionChanged;      // true iff the ctrl mode has just been changed
    DSemaphore               actionSem;          // semaphore managing the access to ctrlLaw and ctrlLawChanged

    // ACTIONS
    map<Action, IAction*>   actions;            // available actions
    double                  feasibility;        // index assessing the feasibility of the current task (0 unfeasible, 1 perfectly feasible)

    RobotStatus         robot;          // current robot status (q, torques, wrench)
    Vector              x;              // position of ctrl point (root ref frame)
    CtrlRef             ctrlRef;        // controller references (desired cart force, cart position, joint torques, joint pos)
    Vector              fd;             // desired contact force/torque
    Vector              xd;             // desired position of ctrl point (root ref frame)
    bool                isRefStable;    // true if the ctrlRef are stable
	
    void sendMsg(const string &msg, const MsgType &type=MSG_INFO) throw();
    void sanityCheck() throw();
    void updateRobotStatus() throw();
    void initAction(const Action &cp) throw();
    ActionStatus executeAction(const Action &cp) throw();
    void releaseAction(const Action &cp) throw();
    void sendMonitorData(const Action &cp) throw();
public:	
    enum IPlannerCommand{get_action, stop_action, line_scrub_action, adaptive_line_scrub_action, circle_action, IPlannerCommandSize};

    controlPlanner(string _moduleName, IController* _controller, int _period, VerbosityLevel _verbose=NO_VERBOSE) throw();
	
    bool threadInit();	
    void run();
    void threadRelease();

    // *******************************************************************************************************
    // *                                              SET METHODS                                            *
    // *******************************************************************************************************
   
    inline Status setParameter(std::string key, const Vector &value, ControlLaw cm=NO_CONTROL) throw(){ 
        return Status();
    }

    inline void setAction(const Action &cp){
        if(action != cp){
            actionSem.wait();
            actionChanged = true;
            pastAction = action;
            action = cp;
            actionSem.post();
        }
    }

    // *******************************************************************************************************
    // *                                              GET METHODS                                            *
    // *******************************************************************************************************
    inline Action getAction(){ return action; }
	inline Status getThreadStatus(){ return thread_status; }
    inline Status getParameter(std::string key, Vector &value){ 
        return Status();
    }

    virtual iCub::skinForceControl::Status respond(const Bottle& command, Bottle& reply){
        Status s = actions[getAction()]->respond(command, reply);
        if(s)
            return s;
        unsigned int cmdId;
        Bottle params;
        if( ! identifyCommand(command, commandList, cmdId, params))
            return Status("Unknown command: "+string(command.toString().c_str()));
    
        switch(cmdId){
        case get_action:
            reply.addString(Action_desc[getAction()].c_str()); return Status();
        case stop_action:
            setAction(NO_ACTION); break;
        case line_scrub_action:
            setAction(LINE_SCRUB); break;
        case adaptive_line_scrub_action:
            setAction(ADAP_LSCRUB); break;
        case circle_action:
            setAction(CIRCLE); break;
        default: 
            return Status("Command not managed");
        }
        reply.addString( (commandList[cmdId]+" command received.").c_str());
        
        return Status();
    }

    inline vector<string> getCommandList() {
        vector<string> cl = commandList;
        vector<string> cla = actions[getAction()]->getCommandList();
        cl.insert(cl.end(), cla.begin(), cla.end());
        return cl; 
    }

    inline vector<string> getCommandDesc() {
        vector<string> cd = commandDesc;
        vector<string> cda = actions[getAction()]->getCommandDesc();
        cd.insert(cd.end(), cda.begin(), cda.end());
        return cd;
    }
};


}

} // end namespace

#endif
