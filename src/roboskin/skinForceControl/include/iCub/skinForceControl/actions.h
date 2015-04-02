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

#ifndef ACTIONS
#define ACTIONS

#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/skinForceControl/util.h>
#include <iCub/skinForceControl/controlThread.h>


namespace iCub
{

namespace skinForceControl
{

enum ActionStatus { ONGOING, DONE};
const std::string ActionStatus_s[] = {"ONGOING", "DONE"};

/**
 * Abstract action.
 */
class IAction{
protected:
    std::vector<std::string>    commandList;        // list of commands accepted by the action
    std::vector<std::string>    commandDesc;        // description of commands accepted by the action
    enum IActionCommand{get_status, resetCmd, IActionCommandSize};

    ActionStatus                status;             // status of the action
    std::string                 name;               // complete name of the law (tipically the class name)
    std::string                 shortName;          // short form of the ctrl law name (3/4 characters)
    IController*                controller;         // robot controller
    DSemaphore                  mutex;              // mutex
    unsigned int                period;             // action execution period
    
    /**
     * Execute the action.
     * @return the action status.
     * @note this method is called by the execute method after taking the semaphore and 
     *       has to be implemented in the actual action that inherit from this class
     */
    virtual ActionStatus abstractExecute()=0;

    virtual void init(){
        name = "IAction";
        shortName = "act";
        mutex.setName(name+"Mutex");
        status = ONGOING;
        commandList.push_back("get status");
        commandDesc.push_back("get the status of the current action (DONE, ONGOING)");
        commandList.push_back("reset");
        commandDesc.push_back("reset the current action");
    }
public:

    /**
     * Constructor.
     * @_controller robot controller
     * @_period action period in ms
     */
    IAction(IController* _controller, int _period);

    /**
     * Default constructor.
     */
    IAction();

    /**
     * Initialize the action. This method should be called before the first call to "executeAction".
     */
    virtual Status initAction() = 0;

    /**
     * Execute the action.
     * @return the status of the operation
     * @note Actually this method just acquires the mutex, calls abstractExecute and releases the mutex.
     */
    ActionStatus executeAction();

    /**
     * Release the action. This method should be called after the last call to "executeAction".
     */
    virtual Status releaseAction() = 0;

    /**
     * Reset the action status.
     */
    Status reset();
    ActionStatus getStatus() {   return status; }
    std::string getName() {     return name; }
    std::string getShortName() { return shortName; }

    virtual Status respond(const Bottle& command, Bottle& reply);
    virtual Status respond(unsigned int &cmdId, Bottle &params, Bottle& reply);
    std::vector<std::string> getCommandList() const { return commandList; }
    std::vector<std::string> getCommandDesc() const { return commandDesc; }
};


/**
 * This action does nothing
 */
class NoAction: public IAction{
protected:

    virtual ActionStatus abstractExecute();
    virtual void init();
public:
    NoAction(IController* _controller, int _period);

    virtual Status initAction();
    virtual Status releaseAction();
};


/**
 * This action uses parallel control to make the robot scrub against the environment.
 * The path taken by the contact point is a line.
 * The robot is assumed to be already in contact with the environment when the action starts.
 * Unless something goes wrong, the action is endless.
 */
class LineScrub: public IAction{
protected:
    enum LineScrubCommand { 
        get_scrub_force=IActionCommandSize, set_scrub_force, 
        get_scrub_path, set_scrub_path, get_alphaFdir, set_alphaFdir, get_scrub_vel, set_scrub_vel,
        dynamics_on, dynamics_off, LineScrubCommandSize
    };
    enum LineScrubPhase{ scrub_back, scrub_forth};

    yarp::sig::Vector xd, xd_link;
    unsigned int xdLinkNum;
    yarp::sig::Vector scrubForce;   // force to apply when scrubbing
    yarp::sig::Vector scrubPath;    // path to follow when scrubbing
    yarp::sig::Vector scrubVel;     // scrubbing velocity
    LineScrubPhase currentPhase;    // scub phase (either back or forth)
    double alphaFdir;               // desired force direction low pass filter intensity (in [0, 1])
    bool dynOn;                     // true if dynamics compensation is on
    

    virtual ActionStatus abstractExecute();
    virtual void init(){
        name = "LineScrub";
        shortName = "lnscr";
        mutex.setName(name+"Mutex");
        currentPhase = scrub_back;    //so that it starts going forth
        alphaFdir = 0.0;
        dynOn = true;
        commandList.push_back("get scrub force");
        commandDesc.push_back("get the desired force to apply when scrubbing");
        commandList.push_back("set scrub force");
        commandDesc.push_back("set the desired force to apply when scrubbing");
        commandList.push_back("get scrub path");
        commandDesc.push_back("get the desired relative path to follow when scrubbing");
        commandList.push_back("set scrub path");
        commandDesc.push_back("set the desired relative path to follow when scrubbing");
        commandList.push_back("get alphaFdir");
        commandDesc.push_back("get intensity of desired force direction low pass filter");
        commandList.push_back("set alphaFdir");
        commandDesc.push_back("set intensity of desired force direction low pass filter");
        commandList.push_back("get scrub vel");
        commandDesc.push_back("get the desired scrubbing velocity");
        commandList.push_back("set scrub vel");
        commandDesc.push_back("set the desired scrubbing velocity");
        commandList.push_back("line dyn on");
        commandDesc.push_back("use dynamics compensation");
        commandList.push_back("line dyn off");
        commandDesc.push_back("do not use dynamics compensation");
    }
public:
    /**
     * Constructor.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     * @param _scrubForce desired force to apply on the environment while scrubbing
     * @param _scrubPath relative target position to reach for scrubbing
     */
    LineScrub(IController* _controller, int _period, const yarp::sig::Vector& _scrubForce, const yarp::sig::Vector& _scrubPath);

    /**
     * Constructor using default scrub force and scrub path.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     */
    LineScrub(IController* _controller, int _period);

    virtual Status initAction();
    virtual Status releaseAction();

    virtual Status respond(const Bottle& command, Bottle& reply);
    virtual Status respond(unsigned int &cmdId, Bottle &params, Bottle& reply);
    virtual Status setScrubForce(const Bottle& params);
    virtual Status setScrubPath(const Bottle& params);
    virtual Status setScrubVel(const Bottle& params);
    virtual Status setAlphaFdir(const Bottle& params);
};


/**
 * A line scrub that modifies the desired position based on the force error.
 * Note that only the final xd is adapted, while the trajectory is still
 * generated by the low pass filter of the controller.
 */
class AdaptiveLineScrub: public LineScrub{
protected:
    double eta;

    enum AdaptiveLineScrubCommand { 
        get_eta=LineScrubCommandSize, set_eta, AdaptiveLineScrubCommandSize
    };

    virtual ActionStatus abstractExecute();
    virtual void init(){
        name = "AdaptiveLineScrub";
        shortName = "alnscr";
        mutex.setName(name+"Mutex");
        commandList.push_back("get eta");
        commandDesc.push_back("get the learning rate");
        commandList.push_back("set eta");
        commandDesc.push_back("set the learning rate");
    }
public:
    /**
     * Constructor.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     * @param _scrubForce desired force to apply on the environment while scrubbing
     * @param _scrubPath relative target position to reach for scrubbing
     * @param _eta the learning rate used to adapt the desired position
     */
    AdaptiveLineScrub(IController* _controller, int _period, const yarp::sig::Vector& _scrubForce, const yarp::sig::Vector& _scrubPath, const double& _eta);

    /**
     * Constructor using default values for all the parameters.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     */
    AdaptiveLineScrub(IController* _controller, int _period);

    virtual Status initAction();
    virtual Status releaseAction();

    virtual Status respond(const Bottle& command, Bottle& reply);
    virtual Status respond(unsigned int &cmdId, Bottle &params, Bottle& reply);
    virtual Status setEta(const double& _eta);
};


/**
 * This action uses parallel control to make the robot's end-effector 
 * move in a circular path in the xy plane, around its starting position.
 */
class Circle: public IAction{
protected:
    enum CircleCommand { 
        get_circle_ray=IActionCommandSize, set_circle_ray, get_circle_vel, set_circle_vel, 
        dynamics_on, dynamics_off, CircleCommandSize
    };

    double circleRay;           // ray of the circular path in m
    double pathVel;             // path velocity in m/s
    double currentAngle;        // current angle in rad
    yarp::sig::Vector center;   // center of the circular path
    bool    dynOn;              // true if use dynamics compensation

    virtual ActionStatus abstractExecute();
    virtual void init(){
        name = "Circle";
        shortName = "cir";
        mutex.setName(name+"Mutex");
        dynOn = true;
        commandList.push_back("get circle ray");
        commandDesc.push_back("get the ray of the circular path");
        commandList.push_back("set circle ray");
        commandDesc.push_back("set the ray of the circular path");
        commandList.push_back("get circle vel");
        commandDesc.push_back("get the speed of the circular path");
        commandList.push_back("set circle vel");
        commandDesc.push_back("set the speed of the circular path");
        commandList.push_back("circle dyn on");
        commandDesc.push_back("use dynamics compensation");
        commandList.push_back("circle dyn off");
        commandDesc.push_back("do not use dynamics compensation");
    }
public:
    /**
     * Constructor.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     * @param _circleRay ray of the circular path in m
     * @param _pathVel desired end-effector velocity in m/s
     */
    Circle(IController* _controller, int _period, double _circleRay, double _pathVel);

    /**
     * Constructor using default scrub force and scrub path.
     * @param _controller controller of the robot
     * @param _period period of the action execution
     */
    Circle(IController* _controller, int _period);

    virtual Status initAction();
    virtual Status releaseAction();

    virtual Status respond(const Bottle& command, Bottle& reply);
    virtual Status respond(unsigned int &cmdId, Bottle &params, Bottle& reply);
    virtual Status setCircleRay(const Bottle& params);
    virtual Status setCircleVel(const Bottle& params);
};

}

} // end namespace



#endif
