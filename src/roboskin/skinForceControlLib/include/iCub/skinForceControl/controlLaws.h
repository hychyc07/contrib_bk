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

#ifndef CTRL_LAWS
#define CTRL_LAWS

#include <yarp/os/Semaphore.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/skinForceControl/util.h>
#include <iCub/skinForceControl/skinForceControlLib.h>

namespace iCub
{

namespace skinForceControl
{

/**
 * Reference value(s) for a robot controller
 */
struct CtrlRef{
    yarp::sig::Vector q;            // desired joint angles (deg)
    yarp::sig::Vector dq;           // desired joint velocities (deg/s)
    yarp::sig::Vector ddq;          // desired joint accelerations (deg/s^2)
    
    yarp::sig::Vector torques;      // desired joint torques (Nm)
    
    yarp::sig::Vector xRef;         // desired cartesian position of control point (m)
    yarp::sig::Vector x;            // desired cartesian position of control point filtered by MinJerkTrajGen (m)
    yarp::sig::Vector dx;           // desired cartesian velocity of control point obtained by MinJerkTrajGen (m/s)
    yarp::sig::Vector ddx;          // desired cartesian acceleration of control point obtained by MinJerkTrajGen (m/s^2)
    
    yarp::sig::Vector wrenchRef;    // desired cartesian force/moment of control point (N/Nm)
    yarp::sig::Vector wrench;       // desired cartesian force/moment of control point filtered by FirstOrderLowPassFilter(N/Nm)

    unsigned int linkNumber;        // control point link number
    yarp::sig::Vector ctrlPoint;    // control point position w.r.t. link reference frame filtered by MinJerkTrajGen(m)
    yarp::sig::Vector ctrlPointRef; // control point position w.r.t. link reference frame (m)

    // the second ctlr point is the contact point and it may be used for force ctrl
    unsigned int linkNumber2;       // second control point link number
    yarp::sig::Vector ctrlPoint2;   // second control point position w.r.t. link reference frame (m)

    double pinvDamp;
    yarp::sig::Vector dqMax;        // max joint velocities
};


/**
 * Status of the robot (to use as controller feedback)
 */
struct RobotStatus{
    yarp::sig::Vector q;            // joint angles (deg)
    yarp::sig::Vector dq;           // joint vel (deg/s)
    yarp::sig::Vector ddq;          // joint acc (deg/s^2)

    yarp::sig::Vector torques;      // joint torques (Nm)
    yarp::sig::Vector wrench;       // control point cartesian force/moment (N/Nm)
    yarp::sig::Vector wrenchSmooth; // control point cartesian force/moment after strong low-pass filter (N/Nm)
    yarp::sig::Vector extFtSens;    // external wrench seen at the F/T sensor (N/Nm)
    
    yarp::sig::Vector contactNormal;    // normal to the contact area
    double contactPressure;             // contact pressure

    iCub::skinDynLib::skinContactList contactList;

    yarp::sig::Vector w0;       // angular velocity of the chain base frame
    yarp::sig::Vector dw0;      // angular acceleration of the chain base frame
    yarp::sig::Vector d2p0;     // linear acceleration of the chain base frame

    yarp::sig::Vector activeJoints;   // 0 if the joint is blocked, 1 if it is active
};

// the order of the command in this list MUST correspond to the order of the enum IControlLawCommand
const std::string IControlLawCommand_s[]  = {
    "set kp",               "get kp",
    "set ki",               "get ki",
    "set kd",               "get kd" 
};
// the order in IControlLawCommand_desc must correspond to the order in IControlLawCommand_s
const std::string IControlLawCommand_desc[]  = {
	"set proportional gains of current control law (double)",
    "get proportional gains of current control law (double)",
    "set integral gains of current control law (double)",
    "get integral gains of current control law (double)",
    "set derivative gains of current control law (double)",
    "get derivative gains of current control law (double)"
};

// the order of the command in this list MUST correspond to the order of the enum JointBoundControlLawCommand
const std::string JointBoundControlLawCommand_s[]  = {"set kj", "get kj", "set act", "get act"};
// the order in JointBoundControlLawCommand_desc must correspond to the order in JointBoundControlLawCommand_s
const std::string JointBoundControlLawCommand_desc[]  = {
	"set joint bound avoidance gains of current control law (double)",
    "get joint bound avoidance gains of current control law (double)",
    "set joint bound activation of current control law (double)",
    "get joint bound activation of current control law (double)"};

// the order of the command in this list MUST correspond to the order of the enum ParallelControlLawCommand
const std::string ParallelControlLawCommand_s[]  = {
    "set kf", "get kf", "set kv", "get kv", 
    "set kp2", "get kp2"
};
// the order in ParallelControlLawCommand_desc must correspond to the order in ParallelControlLawCommand_s
const std::string ParallelControlLawCommand_desc[]  = { 
    "set proportional force gains of current control law (double)",
    "get proportional force gains of current control law (double)",
    "set cartesian velocity gains of current control law (double)",
    "get cartesian velocity gains of current control law (double)",
    "set proportional gain of secondary task (double)",
    "get proportional gain of secondary task (double)"
};

// the order of the command in this list MUST correspond to the order of the enum ContactControlLawCommand
const std::string ContactControlLawCommand_s[]  = {
    "set kf", "get kf", "set kv", "get kv",
    "set cont time", "get cont time", "set kp2", "get kp2", 
    "set mono", "get mono", "set fprio", "get fprio", "set kc", "get kc"
};
// the order in ContactControlLawCommand_desc must correspond to the order in ContactControlLawCommand_s
const std::string ContactControlLawCommand_desc[]  = { 
    "set proportional force gains of current control law (double)",
    "get proportional force gains of current control law (double)",
    "set cartesian velocity gains of current control law (double)",
    "get cartesian velocity gains of current control law (double)",
    "set contact trajectory time (double)",
    "get contact trajectory time (double)",
    "set proportional gain of secondary posture task (double)",
    "get proportional gain of secondary posture task (double)",
    "if true control only to to move to D, otherwise control B to move to C and C to move to D",
    "get mono",
    "set force priority", 
    "get force priority", 
    "set joint dynamics compensation gain (premultiply ddq)", 
    "get joint dynamics compensation gain (premultiply ddq)"
};

// the order of the command in this list MUST correspond to the order of the enum SafeReachRigidControlLawCommand_s
const std::string SafeReachRigidControlLawCommand_s[]  = {
    "set kf", "get kf", "set kv", "get kv", "set krest", "get krest", "set clType", "get clType"
};
// the order in SafeReachRigidControlLawCommand_desc must correspond to the order in SafeReachRigidControlLawCommand_s
const std::string SafeReachRigidControlLawCommand_desc[]  = { 
    "set proportional force gains of current control law (double)",
    "get proportional force gains of current control law (double)",
    "set cartesian velocity gains of current control law (double)",
    "get cartesian velocity gains of current control law (double)",
    "set joint posture gains of current control law (double)",
    "get joint posture gains of current control law (double)",
    "set clType (0 rigid, 1 ortho, 2 soft)",
    "get clType (0 rigid, 1 ortho, 2 soft)"
};

/**
 * Abstract PID control law. All the control laws inherit from this class.
 */
class IControlLaw{
protected:
    std::vector<std::string>    commandList;        // list of commands accepted by the ctrl law
    std::vector<std::string>    commandDesc;        // description of commands accepted by the ctrl law

    // PID PARAMETERS
    static const int DEFAULT_TORQUE_SAT_LIM_DOWN    = -100;  // saturation torque down
    static const int DEFAULT_TORQUE_SAT_LIM_UP      =  100;  // saturation torque up
    static const int DEFAULT_N                      =  10;  // derivative low-pass filter bandwidth (3 to 20, typ. 10).
    static const int DEFAULT_TT                     =  (int)1e8; // anti-windup reset time; generally it's 0.1 to 1 times the value of Ti=Kp/Ki, 
                                                            // and greater than Td, but I set it very high so I disable anti-windup
    // NB: I disable anti-windup because otherwise when the pid saturates the integral error accumulates and i don't like it

    iCub::skinForceControl::Status  globalStatus;   // status of the control law
    std::string                     name;           // complete name of the law (tipically the class name)
    std::string                     shortName;      // short form of the ctrl law name (3/4 characters)
    ControlMode                     ctrlMode;       // control mode (either position, velocity or torque)

    iCub::iDyn::iDynChain*          limb;           // kinematic and dynamic representation of the kinematic chain to control
    unsigned int                    N;              // number of joints of the kinematic chain
    yarp::sig::Vector               monitor;        // monitoring data

    DSemaphore                      mutex;          // mutex for the pid access
    iCub::ctrl::parallelPID*        pid;            // parallel pid controller
    unsigned int                    pidN;           // size of the pid gain vectors
    unsigned int                    period;         // controller period in ms

    virtual void construct(iCub::iDyn::iDynLimb* _limb);
    virtual void construct(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
        const yarp::sig::Vector &_kd);
    virtual void initPid(int period, const yarp::sig::Vector &kp, const yarp::sig::Vector &ki, const yarp::sig::Vector &kd, 
                        const yarp::sig::Vector &N, const yarp::sig::Vector &Tt, const yarp::sig::Matrix &satLim);
    /**
     * Compute the control law output.
     * @param ref the reference to track. 
     * @param s the robot status (i.e. plant feedback). 
     * @return the PID output.
     * @note this method is called by the compute method after taking the semaphore and 
     *       has to be implemented in the actual control laws that inherit from this class
     */
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility)=0;

    virtual iCub::skinForceControl::Status setParameter(const std::string &key, const yarp::sig::Vector &value);
    virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v);
    
public:
    enum IControlLawCommand{ set_kp, get_kp, set_ki, get_ki, set_kd, get_kd, IControlLawCommandSize};

    /**
     * Constructor with PID.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains
     * @_ki integral gains
     * @_kd derivative gains
     */
    IControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
                const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd){ construct(_limb, _period, _kp, _ki, _kd); }

    /**
     * Constructor without PID.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     */
    IControlLaw(iCub::iDyn::iDynLimb* _limb){ construct(_limb); }

    /**
     * Default constructor.
     */
    IControlLaw(){ construct(NULL); }

    /**
     * Compute the control law output.
     * @param ref the reference to track. 
     * @param s the robot status (i.e. plant feedback).
     * @param out the torque resulting from the control law
     * @param feasibility positive value in [0, 1] assessing the feasibility of the task
     * @return the status of the operation
     * @note Actually this method just acquires the mutex, calls abstractCompute and releases the mutex.
     */
    iCub::skinForceControl::Status compute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    iCub::skinForceControl::Status compute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out){
        double f;
        return compute(ref, s, out, f);
    }

    /**
     * Reset the control law internal status (e.g. integral and derivative term of the PID).
     * Method called every time the control law is changed, before calling compute.
     */
    virtual iCub::skinForceControl::Status reset(const CtrlRef &ref, const RobotStatus &s);

    /**
     * Manage the changes of the references. Method called every time a reference is changed.
     * @param ref Reference containing the new values
     * @param s Current status of the robot
     */
    virtual void referenceChanged(const CtrlRef &ref, const RobotStatus &s){}

    /**
     * Get the status of the control law.
     */
    iCub::skinForceControl::Status getStatus(){ return globalStatus; }
    iCub::skinForceControl::ControlMode getControlMode(){ return ctrlMode; }
    yarp::sig::Vector getMonitorData(){ return safeGet(monitor, mutex); }

    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp){ return setParameter("Kp", _kp); }
    virtual iCub::skinForceControl::Status setKi(const yarp::sig::Vector &_ki){ return setParameter("Ki", _ki); }
    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd){ return setParameter("Kd", _kd); }
    virtual yarp::sig::Vector   getKp(){ yarp::sig::Vector v; getParameter("Kp", v); return v;}
    virtual yarp::sig::Vector   getKi(){ yarp::sig::Vector v; getParameter("Ki", v); return v; }
    virtual yarp::sig::Vector   getKd(){ yarp::sig::Vector v; getParameter("Kd", v); return v; }
    
    virtual std::string         getName(){      return name;        }
    virtual std::string         getShortName(){ return shortName;   }
    virtual unsigned int        getPidN()   const{ return pidN; }
    virtual unsigned int        getN()      const{ return N;    }
    
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
    std::vector<std::string> getCommandList() const { return commandList; }
    std::vector<std::string> getCommandDesc() const { return commandDesc; }
};


/**
 * A joint bound avoidance control law. It tries to make the joints not to hit their bounds.
 * If a joint is not close to its bound the control law commands a zero torque, 
 * otherwise it commands a torque that moves the joint away from the bound.
 * The method mergeJntBoundCtrl joins together the torques commanded by another ctrl law
 * (tipically the main task) and the joint bound avoidance: higher priority is given to
 * the joint bound avoidance (this introduces discontinuity => jerky robot movements).
 */
class JointBoundControlLaw: public IControlLaw{
protected:
    yarp::sig::Vector kj;                       // module of the torque applied to move each joint away form the bounds
    yarp::sig::Vector activationThresholds;     // distance from the bound at which the control starts
    yarp::sig::Vector deactivationThresholds;   // distance from the bound at which the control stops
    yarp::sig::Vector jointOverBound;           // 1 if joint is close to bound, 0 otherwise
    yarp::sig::Vector qOnMin, qOnMax, qOffMin, qOffMax; // internal thresholds

    // Just call computeJntBoundCtrl
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility){
        feasibility = 1.0;
        return computeJntBoundCtrl(s, out);
    }

    /**
     * Compute the control torque for maintaining the joints away from their bounds.
     */
    virtual iCub::skinForceControl::Status computeJntBoundCtrl(const RobotStatus &s, yarp::sig::Vector &out);

    /**
     * Merge together the torque coming from the control law with the torque coming from the joint bound avoidance.
     * If the ctrlTorque has the same sign as the jbTorque then they are summed.
     * If they have different sign the jbTorque is used.
     * The output is written in ctrlTorque.
     * @param jbTorque torque necessary to push the joint away from a close bound
     * @param ctrlTorque torque to perform the current task
     * @note both torques have to be without gravity compensation
     */
    virtual void mergeJntBoundCtrl(yarp::sig::Vector &jbTorque, yarp::sig::Vector &ctrlTorque);

    void updateInternalThresholds();

    /**
     * Empty constructor.
     */
    JointBoundControlLaw();

    /**
     * Protected constructor with no joint bound control.
     */
    JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd);

    /**
     * Protected constructor with joint bound control.
     */
    JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    void init(const yarp::sig::Vector &_kj, 
                const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

public:
    enum JointBoundControlLawCommand{ set_kj=IControlLawCommandSize, get_kj, set_act, get_act, JointBoundControlLawCommandSize};

    /**
     * Constructor.
     * @param _limb the kinematic chain to control
     * @param _kj gain used to compute the torque to move the joints away from their bounds
     * @param _activationThresholds distance (in deg) to the joint bound at which the ctrl law starts
     * @param _deactivationThresholds distance (in deg) to the joint bound at which the ctrl law stops (after it had started)
     */
    JointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    //virtual iCub::skinForceControl::Status setParameter(const std::string &key, const yarp::sig::Vector &value);
    virtual iCub::skinForceControl::Status setKj(const yarp::sig::Vector &_kj);
    virtual iCub::skinForceControl::Status setActivationThresholds(const yarp::sig::Vector &_activationThresholds);
    virtual iCub::skinForceControl::Status setDeactivationThresholds(const yarp::sig::Vector &_deactivationThresholds);

    //virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v) const;
    virtual yarp::sig::Vector getKj(){ return safeGet(kj, mutex); }
    virtual yarp::sig::Vector getActivationThresholds(){ return safeGet(activationThresholds, mutex); }
    virtual yarp::sig::Vector getDeactivationThresholds(){ return safeGet(deactivationThresholds, mutex); }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return IControlLaw::respond(command, reply);
    }
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
};

/**
 * A joint bound avoidance control law. It tries make the joints not hit their bounds.
 * An exponential repulsive force field is simulated at each joint bound and that may be 
 * summed to the main task ctrl torque through the method mergeJntBoundCtrl.
 * Note that if the main task torque is higher than the force field torque the joint could
 * hit the bound anyway, so this ctrl law is less safe than the JointBoundControlLaw, but 
 * it has the assett to be continuous (robot movements should not be jerky).
 * Distance to joint bound => force field intensity
 * 2a  => 2% 
 * a   => 37%
 * a/2 => 78%
 * where a is the activation threshold.
 */
class ForceFieldJointBoundControlLaw: public JointBoundControlLaw{
protected:
    yarp::sig::Vector jointBoundMin;
    yarp::sig::Vector jointBoundMax;

    // Just call computeJntBoundCtrl
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility){
        feasibility = 1.0;
        return computeJntBoundCtrl(s, out);
    }

    /**
     * Compute the control torque for maintaining the joints away from their bounds.
     */
    virtual iCub::skinForceControl::Status computeJntBoundCtrl(const RobotStatus &s, yarp::sig::Vector &out);

    /**
     * Just sum the torque coming from the control law with the torque coming from the joint bound avoidance.
     * @param jbTorque torque necessary to push the joint away from a close bound
     * @param ctrlTorque torque to perform the current task
     */
    virtual void mergeJntBoundCtrl(yarp::sig::Vector &jbTorque, yarp::sig::Vector &ctrlTorque){ yarp::math::operator+=(ctrlTorque, jbTorque); }

    /**
     * Empty constructor.
     */
    ForceFieldJointBoundControlLaw();

    /**
     * Protected constructor with no joint bound control.
     */
    ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd);

    /**
     * Protected constructor with joint bound control.
     */
    ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual void init();

public:
    /**
     * Constructor.
     * @param _limb the kinematic chain to control
     * @param _kj gain used to compute the torque to move the joints away from their bounds
     * @param _activationThresholds distance (in deg) to the joint bound at which the ctrl law starts
     * @param _deactivationThresholds distance (in deg) to the joint bound at which the ctrl law stops (after it had started)
     */
    ForceFieldJointBoundControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual iCub::skinForceControl::Status setActivationThresholds(const yarp::sig::Vector &_activationThresholds);
};


/**
 * Joint torque control law.
 * Actually this control law is trivial because it simply returns the reference torques,
 * but it is necessary to treat the torque control as any other control law.
 */
class TorqueControlLaw: public ForceFieldJointBoundControlLaw{
protected:
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    virtual void init();
public:
    /**
     * Constructor without joint bound control. Since this class is really dummy this constructor does nothing.
     */
    TorqueControlLaw(){ init(); }

    /**
     * Constructor with joint bound control.
     * @param _limb the kinematic chain to control
     * @param _kj gain used to compute the torque to move the joints away from their bounds
     * @param _activationThresholds distance (in deg) to the joint bound at which the ctrl law starts
     * @param _deactivationThresholds distance (in deg) to the joint bound at which the ctrl law stops (after it had started)
     */
    TorqueControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);
};


/**
 * Float control law.
 * Just compensates for the gravity.
 */
class FloatControlLaw: public ForceFieldJointBoundControlLaw{
protected:
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    /**
     * Compute the control torque for gravity compensation.
     */
    virtual iCub::skinForceControl::Status computeGravityCompensation(const RobotStatus &s, yarp::sig::Vector &out);

    virtual void init();

    /**
     * Protected constructor without joint bound control.
     */
    FloatControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
        const yarp::sig::Vector &_kd);

    /**
     * Protected constructor with joint bound control.
     */
    FloatControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

public:
    enum FloatControlLawCommand { FloatControlLawCommandSize=JointBoundControlLawCommandSize };
    /**
     * Constructor without joint bound control.
     */
    FloatControlLaw(iCub::iDyn::iDynLimb* _limb);

    /**
     * Constructor with joint bound control.
     * @param _limb the kinematic chain to control
     * @param _kj gain used to compute the torque to move the joints away from their bounds
     * @param _activationThresholds distance (in deg) to the joint bound at which the ctrl law starts
     * @param _deactivationThresholds distance (in deg) to the joint bound at which the ctrl law stops (after it had started)
     */
    FloatControlLaw(iCub::iDyn::iDynLimb* _limb, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);
};


/**
 * Joint position control law based on a PID + gravity compensation.
 * This law controls the joint angles of a robotic chain.
 */
class JPositionControlLaw: public FloatControlLaw{
protected:
    yarp::sig::Vector kp;
    yarp::sig::Vector kd;

    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);
    void init();
public:
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains
     * @_ki integral gains
     * @_kd derivative gains
     */
    JPositionControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd);

    /**
     * Constructor with joint bound control.
     */
    JPositionControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp){ 
        if(_kp.size()!=kp.size())
            return Status("Error while setting param Kp, expected size: "+toString(kp.size())+"; found size: "+toString(_kp.size()));
        mutex.wait(); kp = _kp; mutex.post(); return Status(); 
    }
    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd){ 
        if(_kd.size()!=kd.size())
            return Status("Error while setting param Kd, expected size: "+toString(kd.size())+"; found size: "+toString(_kd.size()));
        mutex.wait(); kd = _kd; mutex.post(); return Status(); 
    }
    virtual yarp::sig::Vector   getKp(){ return kp; }
    virtual yarp::sig::Vector   getKd(){ return kd; }
};


/**
 * Cartesian position control law based on a PID + gravity compensation.
 * This law controls the cartesian position (ONLY POSITION!, not orientation) of a point of the robot.
 */
class PositionControlLaw: public FloatControlLaw{
protected:
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);
    void init();
public:
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains
     * @_ki integral gains
     * @_kd derivative gains
     */
    PositionControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd);

    /**
     * Constructor with joint bound control.
     */
    PositionControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);
};

/**
 * Cartesian force control law based on a PID + gravity compensation.
 * This law controls the cartesian force (force and optionally moment) exchanged 
 * between the robot and the environment.
 * The derivative term is based on the joint velocities rather then on the derivative of the 
 * cartesian force (which may be very noisy).
 */
class ForceControlLaw: public FloatControlLaw{
protected:
    yarp::sig::Vector   kd;
    bool enableMomentCtrl;

    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);
    void init(const yarp::sig::Vector &_kd);
public:
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains
     * @_ki integral gains
     * @_kd derivative gains
     * @_enableMomentCtrl if false, control force but not moment, otherwise control both force and moment
     */
    ForceControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd, bool _enableMomentCtrl=true);

    /**
     * Constructor with joint bound control.
     */
    ForceControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, 
        const yarp::sig::Vector &_ki, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds,
        bool _enableMomentCtrl=true);

    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd);
    virtual yarp::sig::Vector getKd(){ return safeGet(kd, this->mutex); }
};


/**
 * Parallel control law (Chiaverini, Sciavicco) with gravity compensation.
 * This law controls both the cartesian position (ONLY POSITION, not orientation) and the 
 * cartesian force (ONLY FORCE, not moment) of a specific point on the robotic chain.
 * The position term is computed with a PD, while the force term with a PI, so that higher
 * priority is given to the force feedback.
 */
class ParallelControlLaw: public FloatControlLaw{
protected:
    yarp::sig::Vector   kd;                 // joint position derivative gains
    yarp::sig::Vector   kp;                 // cartesian position proportional gains
    yarp::sig::Vector   kv;                 // cartesian velocity gains
    yarp::sig::Vector   kp2;                // joint proportional gain for secondary task

    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);
    void init();
public:
    enum ParallelControlLawCommand { 
        set_kf=FloatControlLawCommandSize, get_kf, set_kv, get_kv, 
        set_kp2, get_kp2, ParallelControlLawCommandSize
    };
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains for the position control loop
     * @_ki integral gains for the force control loop
     * @_kd derivative gains for the position control loop (using joint velocities, not cartesian position derivative)
     * @_kf proportional gains for the force control loop
     * @note kp, ki and kf are 3-dim vectors, while kd is an n-dim vector (where n=robotic chain DOFs)
     */
    ParallelControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv);

    /**
     * Constructor with joint bound control.
     */
    ParallelControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
        const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual iCub::skinForceControl::Status setParameter(const std::string &key, const yarp::sig::Vector &value);
    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp);
    virtual iCub::skinForceControl::Status setKp2(const yarp::sig::Vector &_kp2);
    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd);
    virtual iCub::skinForceControl::Status setKf(const yarp::sig::Vector &_kf){ return setParameter("Kp", _kf); }
    virtual iCub::skinForceControl::Status setKv(const yarp::sig::Vector &_kv);

    virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v);
    virtual yarp::sig::Vector getKp(){  return safeGet(kp, mutex); }
    virtual yarp::sig::Vector getKp2(){  return safeGet(kp2, mutex); }
    virtual yarp::sig::Vector getKd(){  return safeGet(kd, mutex); }
    virtual yarp::sig::Vector getKf(){  return FloatControlLaw::getKp(); }
    virtual yarp::sig::Vector getKv(){  return safeGet(kv, mutex); }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return JointBoundControlLaw::respond(command, reply);
    }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
};

/**
 * Parallel control law (Chiaverini, Sciavicco) with compensation of the dynamics of the robot.
 */
class DynParallelControlLaw: public ParallelControlLaw{
protected:
    yarp::sig::Vector jb;   // joint bound control
    yarp::sig::Vector ddqL, ddqU;   // joint acceleration lower and upper bounds
    
    yarp::sig::Matrix R_r_c;        // rotation root-contact of pos ctrl pnt
    yarp::sig::Vector ctrlPnt_r;    // pos ctrl point w.r.t. link, in root ref frame
    yarp::sig::Vector x;            // pos ctrl pnt w.r.t. root, in root ref frame
    yarp::sig::Vector dx;           //linear velocity of position ctlr pnt

    yarp::sig::Matrix tempJ, Jl_p, Jl_f;            // J_p=jacobian of pos ctrl pnt, J_f=jacobian of force ctlr pnt
    yarp::sig::Matrix tempdJ, dJl_p, dJl_f;         // jacobian time derivative
    yarp::sig::Matrix Jl_p_pinv, Jl_f_pinv;         // jacobian pseudo-inverse
    yarp::sig::Matrix N_f;                          // nullspace projection matrix of Jl_f

    yarp::sig::Vector ddx_f, ddx_p;         // desired cartesian accelerations (force PI term, position P term)
    yarp::sig::Vector y_f, y_p;             // ddx-dJ*dq
    yarp::sig::Vector ddq_p, ddq_f;         // desired joint acceleration (pos term and force term)
    yarp::sig::Vector f, fd;                // measured and desired contact force
    
    yarp::sig::Vector z6, z3;           // 6-dim and 3-dim zero vectors

    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    /**
    * Compute the linear jacobian and its time derivative.
    * @param linkNumber the link number
    * @param ctrlPoint the control point expressed w.r.t. its link reference frame
    * @param activeJoints a vector of ones and zeros representing the active joints with 1
    * @param dqRad joint velocities expressed in rad/sec
    * @param Jl the linear part of the jacobian
    * @param dJl the time derivative of the linear part of the jacobian
    */
    virtual void computeJacobians(unsigned int linkNumber, const yarp::sig::Vector &ctrlPoint, const yarp::sig::Vector &activeJoints,
        const yarp::sig::Vector &dqRad, yarp::sig::Matrix &Jl, yarp::sig::Matrix &dJl);

    void init();

public:
    enum DynParallelControlLawCommand { DynParallelControlLawCommandSize=ParallelControlLawCommandSize};
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains for the position control loop
     * @_ki integral gains for the force control loop
     * @_kd derivative gains for the position control loop (using joint velocities, not cartesian position derivative)
     * @_kf proportional gains for the force control loop
     * @note kp, ki and kf are 3-dim vectors, while kd is an n-dim vector (where n=robotic chain DOFs)
     */
    DynParallelControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv);

    /**
     * Constructor with joint bound control.
     */
    DynParallelControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
        const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return ParallelControlLaw::respond(command, reply);
    }
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply){
        return ParallelControlLaw::respond(cmdId, params, reply);
    }
};

/**
 * Contact control law. Control linear and angular acceleration of the contact point in order
 * to move the control point into a specific 3d position. At the same time regulates the
 * contact force.
 */
class ContactControlLaw: public ForceFieldJointBoundControlLaw{
protected:
    yarp::sig::Vector jb;               // joint bound control torques
    yarp::sig::Vector kp, kv, kd;       // proportional, velocity, damping gains
    yarp::sig::Vector kp2;              // secondary posture task gains
    yarp::sig::Vector kc;               // joint dynamics compensation gains
    double forcePriority;               // weight of force control in pseudoinverse (usually >1)

    yarp::sig::Vector B;                // desired robot contact point w.r.t. root, in root ref frame
    yarp::sig::Vector C;                // current robot contact point w.r.t. root, in root ref frame
    yarp::sig::Vector D;                // desired environment contact point w.r.t. root, in root ref frame
    yarp::sig::Vector C_r;              // C w.r.t. link, in root ref frame
    
    yarp::sig::Vector dB;               // linear velocity of B
    yarp::sig::Vector dC;               // linear velocity of C

    yarp::sig::Vector Bref;             // reference position of B, output of minJerkTrajGen
    yarp::sig::Vector Cref;             // reference position of C, output of minJerkTrajGen

    yarp::sig::Vector ddBd;             // desired acceleration of B
    yarp::sig::Vector ddCd;             // desired acceleration of C

    yarp::sig::Matrix H_r_c;            // rototranslation root-contact link
    yarp::sig::Matrix R_r_c;            // rotation root-contact link
    yarp::sig::Vector dqRad;            // joint velocities in rad/sec

    yarp::sig::Matrix Jlink, dJlink;    // jacobian of the contact link
    yarp::sig::Matrix Jc, Jc_l, Jc_a;   // jacobian of C, its linear part and its angular part
    yarp::sig::Matrix Jb, Jd, dJb;      // jacobian of B and (D-C) (linear part only)
    yarp::sig::Matrix dJc, dJc_l, dJc_a;   // jacobian time derivative
    yarp::sig::Vector Jf;               // jacobian of C considering only force direction
    yarp::sig::Matrix J;                // multipoint jacobian considering both B, C and D
    yarp::sig::Matrix J_pinv;           // multipoint jacobian pseudo-inverse

    double ddx_f;                       // linear acceleration due to force control
    yarp::sig::Vector y, yB, yC;        // desired J*ddq
    yarp::sig::Vector ddq;              // desired joint accelerations due to both force and position control
    yarp::sig::Vector fd, f, fDir;
    yarp::sig::Vector z6, z3;           // 6-dim and 3-dim zero vectors

    // for monitoring
    yarp::sig::Vector ddB, ddC, dwC;
    double ddf;

    double trajTime;
    iCub::ctrl::minJerkRefGen* trajGen_B;          // desired position of B w.r.t. B min-jerk filter
    iCub::ctrl::minJerkTrajGen* trajGen_C;          // desired (D-C) min-jerk filter

    bool monoTask;

    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    /**
    * Compute the jacobian, its time derivative and its pseudoinverse.
    * @param linkNumber the link number
    * @param ctrlPoint the control point expressed w.r.t. its link reference frame
    * @param activeJoints a vector of ones and zeros representing the active joints with 1
    * @param dqRad joint velocities expressed in rad/sec
    * @param damping pseudoinverse damping factor
    */
    virtual void computeJacobians(const CtrlRef &ref, const RobotStatus &s);

    void sendMonitorData();

    void init();

public:
    enum ContactControlLawCommand { 
        set_kf=JointBoundControlLawCommandSize, get_kf, set_kv, get_kv, 
        set_cont_traj_time, get_cont_traj_time, set_kp2, get_kp2, set_mono, get_mono,
        set_force_priority, get_force_priority, set_kc, get_kc,
        ContactControlLawCommandSize
    };

    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains for the position control loop
     * @_ki integral gains for the force control loop
     * @_kd damping gains for the joint damping
     * @_kf proportional gains for the force control loop
     * @_kv cartesian velocity gains for position control
     * @_kr rotational gain
     * @_trajTime trajectory time of minimum jerk trajectory generators
     * @note kp, kv, ki and kf are 3-dim vectors, while kd is an n-dim vector (where n=robotic chain DOFs)
     */
    ContactControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
                    const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv,
                    double _trajTime, double _forcePriority, const yarp::sig::Vector _kc);

    /**
     * Constructor with joint bound control.
     */
    ContactControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_ki, 
        const yarp::sig::Vector &_kd, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kv, 
        double _trajTime, double _forcePriority, const yarp::sig::Vector _kc, const yarp::sig::Vector &_kj, 
        const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual iCub::skinForceControl::Status reset(const CtrlRef &ref, const RobotStatus &s);
    virtual void referenceChanged(const CtrlRef &ref, const RobotStatus &s);

    virtual iCub::skinForceControl::Status setParameter(const std::string &key, const yarp::sig::Vector &value);
    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp);
    virtual iCub::skinForceControl::Status setKp2(const yarp::sig::Vector &_kp2);
    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd);
    virtual iCub::skinForceControl::Status setKf(const yarp::sig::Vector &_kf){ return setParameter("Kp", _kf); }
    virtual iCub::skinForceControl::Status setKv(const yarp::sig::Vector &_kv);
    virtual iCub::skinForceControl::Status setKc(const yarp::sig::Vector &_kc);
    virtual iCub::skinForceControl::Status setContTrajTime(double _trajTime);
    virtual iCub::skinForceControl::Status setForcePriority(double fp);
    virtual iCub::skinForceControl::Status setMono(bool mono);

    virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v);
    virtual yarp::sig::Vector getKp(){  return safeGet(kp, mutex); }
    virtual yarp::sig::Vector getKp2(){  return safeGet(kp2, mutex); }
    virtual yarp::sig::Vector getKd(){  return safeGet(kd, mutex); }
    virtual yarp::sig::Vector getKf(){  return ForceFieldJointBoundControlLaw::getKp(); }
    virtual yarp::sig::Vector getKv(){  return safeGet(kv, mutex); }
    virtual yarp::sig::Vector getKc(){  return safeGet(kc, mutex); }
    virtual bool getMono(){ return safeGet(monoTask, mutex); }
    virtual double getContTrajTime(){ return safeGet(trajTime, mutex); }
    virtual double getForcePriority(){ return safeGet(forcePriority, mutex); }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return ForceFieldJointBoundControlLaw::respond(command, reply);
    }
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
};

/**
 * Pressure control law. It does not use the F/T sensor, but just the skin.
 */
class PressureControlLaw: public IControlLaw{
protected:
    yarp::sig::Vector kp;           // proportional gains
    yarp::sig::Vector dq;           // joint velocities
    yarp::sig::Matrix tempJ, Jl;    // Jl= linear part of jacobian
    yarp::sig::Matrix Jl_pinv;      // jacobian pseudo-inverse
    yarp::sig::Vector sv_Jl;        // singular values of jacobian

    yarp::sig::Vector z6;
    
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    /**
    * Compute the linear jacobian, its time derivative, its pseudo-inverse and its singular values.
    * @param linkNumber the link number
    * @param ctrlPoint the control point expressed w.r.t. its link reference frame
    * @param activeJoints a vector of ones and zeros representing the active joints with 1
    * @param dqRad joint velocities expressed in rad/sec
    * @param Jl the linear part of the jacobian
    * @param Jl_pinv the damped pseudo-inverse of the linear part of the jacobian
    * @param sv_Jl the singular values of the linear part of the jacobian
    */
    virtual void computeJacobians(unsigned int linkNumber, const yarp::sig::Vector &ctrlPoint, const yarp::sig::Vector &activeJoints,
        yarp::sig::Matrix &Jl, yarp::sig::Matrix &Jl_pinv, yarp::sig::Vector &sv_Jl, double pinvDamp);

    void init();

public:
    enum PressureControlLawCommand { PressureControlLawCommandSize=IControlLawCommandSize};
    /**
     * Constructor without joint bound control.
     * @_limb Kinematic and dynamic description of the robotic chain to control
     * @_period controller period in ms
     * @_kp proportional gains for the position control loop
     * @note kp, ki are 3-dim vectors, while kd is an n-dim vector (where n=robotic chain DOFs)
     */
    PressureControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp);

    
    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp);
    virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v);
    virtual yarp::sig::Vector getKp(){  return safeGet(kp, mutex); }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return IControlLaw::respond(command, reply);
    }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
};


/**
 * Safe reach rigid control law. Control law for reaching with the end-effector while controlling
 * optional contact forces. The contacts are supposed to be rigid.
 */
class SafeReachRigidControlLaw: public ForceFieldJointBoundControlLaw{
protected:
    // control law type
    enum clType {CL_RIGID, CL_ORTHO, CL_SPRING};
    clType cltype;

    yarp::sig::Vector jb;               // joint bound control torques
    yarp::sig::Vector kp, kv, kd;       // proportional, velocity, damping gains
    yarp::sig::Vector krest;            // secondary posture task gains
    yarp::sig::Vector fThreshMax, fThreshMin;      // max and min contact force thresholds
    double currentFThresh;
    
    yarp::sig::Vector x, dx;            // end-effector position and its linear velocity
    yarp::sig::Vector ddx_d;            // desired end-effector acceleration

    yarp::sig::Matrix H_r_c;            // rototranslation root-contact link
    yarp::sig::Matrix R_r_c;            // rotation root-contact link
    yarp::sig::Vector dqRad;            // joint velocities in rad/sec

    yarp::sig::Vector ddq_f;            // desired acceleration for force task
    yarp::sig::Vector ddq_0;            // desired acceleration for secondary posture task
    yarp::sig::Vector ddq_d;            // desired joint accelerations due to both force and position control

    yarp::sig::Vector ctrlPoint_r;      // control point in root reference frame
    yarp::sig::Vector ctrlPoint2_r;     // contact point in root reference frame

    yarp::sig::Matrix Jlink, dJlink;    // jacobian of contact link and its derivative
    yarp::sig::Matrix Jc, dJc;          // jacobian of contact point and its derivative
    yarp::sig::Matrix Jf, dJf;          // jacobian of contact point considering only force direction
    yarp::sig::Matrix J, dJ;            // end-effector jacobian
    yarp::sig::Matrix Jf_pinv;          // contact point jacobian pseudo-inverse
    yarp::sig::Matrix Nf;               // nullspace projector of contact point jacobian
    yarp::sig::Matrix Na;               // nullspace projector of augmented jacobian
    yarp::sig::Matrix JNf;              // J*Nf
    yarp::sig::Matrix JNf_pinv;         // pseudo-inverse of JNf

    yarp::sig::Vector w_d;              // desired contact wrench
    yarp::sig::Vector y;                // desired J*ddq
    yarp::sig::Matrix w;                // matrix containing the contact wrench
    yarp::sig::Vector z6, z3;           // 6-dim and 3-dim zero vectors

    bool isInContact;                   // true if robot is in contact, false otherwise
    bool isSkinInContact;               // true if robot is in contact with the skin, false otherwise

    // temp
    CtrlRef currentRef;
    yarp::sig::Vector svJ, svJNf;
    yarp::sig::Matrix J_pinv;

    iCub::ctrl::minJerkTrajGen *forceRefGen;
    iCub::ctrl::minJerkTrajGen *forceThrGen;
    
    virtual iCub::skinForceControl::Status abstractCompute(const CtrlRef &ref, const RobotStatus &s, yarp::sig::Vector &out, double &feasibility);

    /**
    * Compute the jacobian, its time derivative and its pseudoinverse.
    * @param linkNumber the link number
    * @param ctrlPoint the control point expressed w.r.t. its link reference frame
    * @param activeJoints a vector of ones and zeros representing the active joints with 1
    * @param dqRad joint velocities expressed in rad/sec
    * @param damping pseudoinverse damping factor
    */
    virtual void computeJacobians(const CtrlRef &ref, const RobotStatus &s);

    void sendMonitorData();

    void init();

public:
    enum SafeReachRigidControlLawCommand { 
        set_kf=JointBoundControlLawCommandSize, get_kf, set_kv, get_kv, set_krest, get_krest, set_clType, get_clType,
        SafeReachRigidControlLawCommandSize
    };

    /**
     * Constructor without joint bound control.
     * @param _limb Kinematic and dynamic description of the robotic chain to control
     * @param _period controller period in ms
     * @param _kp proportional gains for the position control loop
     * @param _kv cartesian velocity gains for position control
     * @param _kf proportional gains for the force control loop
     * @param _kd damping gains for the joint damping
     * @param _krest proportional gains for the secondary posture task
     * @param _fThresh lower threshold for the contact force norm
     * @note kp, kv, kf are 3-dim vectors, while kd, krest are n-dim vectors (where n=robotic chain DOFs)
     */
    SafeReachRigidControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp, const yarp::sig::Vector &_kv, 
        const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_krest, double _fThresh);

    /**
     * Constructor with joint bound control.
     */
    SafeReachRigidControlLaw(iCub::iDyn::iDynLimb* _limb, int _period, const yarp::sig::Vector &_kp,
        const yarp::sig::Vector &_kv, const yarp::sig::Vector &_kf, const yarp::sig::Vector &_kd, const yarp::sig::Vector &_krest,
        double _fThresh,
        const yarp::sig::Vector &_kj, const yarp::sig::Vector &_activationThresholds, const yarp::sig::Vector &_deactivationThresholds);

    virtual iCub::skinForceControl::Status reset(const CtrlRef &ref, const RobotStatus &s);
    virtual void referenceChanged(const CtrlRef &ref, const RobotStatus &s);

    virtual iCub::skinForceControl::Status setParameter(const std::string &key, const yarp::sig::Vector &value);
    virtual iCub::skinForceControl::Status setKp(const yarp::sig::Vector &_kp){
        if(_kp.size()!=kp.size()) return Status(false, "Error setting kp: wrong size");
        mutex.wait(); kp=_kp; mutex.post();
        return Status();
    }
    virtual iCub::skinForceControl::Status setKrest(const yarp::sig::Vector &_krest){
        if(_krest.size()!=krest.size())
            return Status("Error while setting param Krest, expected size: "+toString(krest.size())+"; found size: "+toString(_krest.size()));
        mutex.wait(); krest = _krest; mutex.post();
        return Status();
    }
    virtual iCub::skinForceControl::Status setKv(const yarp::sig::Vector &_kv){
        if(_kv.size()!=3) return Status(false, "Error setting kv: wrong size");
        mutex.wait(); kv=_kv; mutex.post();
        return Status();
    }
    virtual iCub::skinForceControl::Status setKd(const yarp::sig::Vector &_kd){
        if(_kd.size()!=kd.size()) return Status(false, "Error setting kd: wrong size");
        mutex.wait(); kd=_kd; mutex.post();
        return Status();
    }
    virtual iCub::skinForceControl::Status setKf(const yarp::sig::Vector &_kf){ return setParameter("Kp", _kf); }
    virtual iCub::skinForceControl::Status setClType(int v){
        if(v<0 || v>2) return Status(false, "Error setting clType: out of range "+toString(v));
        mutex.wait(); cltype=(clType)v; mutex.post();
        return Status();
    }
    
    virtual iCub::skinForceControl::Status getParameter(const std::string &key, yarp::sig::Vector &v);
    virtual yarp::sig::Vector getKp(){  return safeGet(kp, mutex); }
    virtual yarp::sig::Vector getKrest(){  return safeGet(krest, mutex); }
    virtual yarp::sig::Vector getKf(){  return ForceFieldJointBoundControlLaw::getKp(); }
    virtual yarp::sig::Vector getKv(){  return safeGet(kv, mutex); }
    virtual yarp::sig::Vector getKd(){  return safeGet(kd, mutex); }

    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param command the command to execute, optionally followed by parameters
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
        return ForceFieldJointBoundControlLaw::respond(command, reply);
    }
    /**
     * Execute the specified command and store the reply in a Bottle. 
     * If the command is unknown return a negative Status.
     * @param cmdId the id of the command to execute
     * @param params the parameters of the command (if any)
     * @param reply (if the command has been executed) the reply to the command
     * @return true if the command has been executed, false otherwise
     */
    virtual iCub::skinForceControl::Status respond(unsigned int cmdId, yarp::os::Bottle &params, yarp::os::Bottle& reply);
};


}

} // end namespace



#endif
