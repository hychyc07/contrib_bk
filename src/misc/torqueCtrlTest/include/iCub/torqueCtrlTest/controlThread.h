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

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include "iCub/torqueCtrlTest/controlConstants.h"
#include "iCub/torqueCtrlTest/robot_interfaces.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

#define NJ 20

namespace iCub
{

namespace torqueCtrlTest
{

class controlThread: public yarp::os::RateThread
{
private:

    VerbosityLevel          verbose;
    std::string             name;               // thread name
    ControlThreadStatus     thread_status;      // the status of the thread (OK, DISCONNECTED)
                                                // as soon as the status is DISCONNECTED, the module calls stop() on this thread
    ControlMode             ctrlMode;           // the type of control used
    bool                    ctrlModeChanged;    // true iff the ctrl mode has just been changed
    Semaphore               ctrlModeSem;        // semaphore managing the access to ctrlMode and ctrlModeChanged
    BodyPart                bodyPart;           // the body part of the robot to control
    CommandMode             cmdMode;            // SIMULATION: no torques are sent to the motors

    parallelPID*            torquePid;
    AWLinEstimator*         velEstimator;
    minJerkTrajGen*         trajGen;
    FirstOrderLowPassFilter* lpf_torque, *lpf_pwm;
    int velEstWind;                             // max window size for velocity estimation
    double velEstThr;                           // threshold for velocity estimation

    // NETWORK DELAY MANAGEMENT
    double                  timePre;
    vector<double>          durations;
    vector<double>          timestamps;
    vector<string>          durationDesc;

    // PORTS
    BufferedPort<Vector>    *port_torques;      // output port sending the same PWM that is commanded to the motor
    BufferedPort<Vector>    *port_monitor;      // output port sending data for monitoring the controller

    Vector                  monitorData;        // vector sent on the monitor port

    // CONTROL LOOP 
	robot_interfaces*   interfaces;
    int                 jointId;        // index of the controlled joint
    double              alpha;          // low pass filter intensity (in [0, 1])
    Vector              tao, taod;      // current and desired joint torques
    Vector              taoG;           // gravity torques
    Vector              pwm;            // pwm commanded to the robot joint motor
    double              pwmD, pwmRead;  // pwm desired (open ctrl mode), pwm read from the motor control board
    Vector              q, qRef, qDes;  // joint angle
    Vector              dqRef, ddqRef;  // joint vel and acc
    double              dq, dqFirmware; // joint speed
    
    double              pwmAll[NJ], qAll[NJ], dqAll[NJ], taoAll[NJ];   // vectors for reading
    double              pwmTimestamp;

    double kp, kd, ki;                      // PID gains
    double k_tao, k_bemf, k_cp, k_cn;       // gains for feedforward control (pwm = k_tao*tao + k_bemf*dq + k_c*sign(dq))
    Vector taoStic;                         // estimate of the joint stiction torque
    double taoCoulomb;                      // estimate of the joint coulomb friction
    double alphaStic;                       
    double k_stic, k_coul, k_dither, w_dither;  // gains for stiction and dither signal (and dither frequency)
    double M, Md, B, K;                     // inertia (real and desired), damping and stiffness for impedance control
    int    isMoving;
    int    isTorqueErrBig, desMotionDir;    // 1 positive, -1 negative, 0 zero
    double torqueSensStdDev;

    bool sanityCheck();
    void updateRobotStatus();

    void initCtrlMode(const ControlMode &cm);
    void computePwm(const ControlMode &cm);
    void sendPwm(const ControlMode &cm);
    void prepareMonitorData(const ControlMode &cm);
    void updateDurations(int index);

    inline Vector getParameter(string key){
        Bottle b;
        torquePid->getOptions(b);
        int size=1;
        Vector v(size);
        helperPID::getVectorFromOption(b, key.c_str(), v, size);
        return v; 
    }

    inline void setParameter(string key, Vector value, ControlMode cm){
        Bottle b;
        helperPID::addVectorToOption(b, key.c_str(), value);
        torquePid->setOptions(b);
    }

public:	

    controlThread(std::string _moduleName, std::string _robotName, int _period, 
        iCub::skinDynLib::BodyPart _bodyPart, VerbosityLevel _verbose=NO_VERBOSE) ;
	
    bool threadInit();	
    void run();
    void threadRelease();

    // SET METHODS
    inline void setBodyPart(BodyPart _bp){ bodyPart = _bp; }
    //inline void setKp(Vector _kp, ControlMode cm=NO_CONTROL) { setParameter("Kp", _kp, cm); kp=_kp;}
    inline void setKp(double _kp, ControlMode cm=NO_CONTROL) { setParameter("Kp", Vector(1, &_kp), cm); kp=_kp;}
    //inline void setKi(Vector _ki, ControlMode cm=NO_CONTROL) { setParameter("Ki", _ki, cm); }
    inline void setKi(double _ki, ControlMode cm=NO_CONTROL) { setParameter("Ki", Vector(1, &_ki), cm); ki=_ki;}
    //inline void setKd(Vector _kd, ControlMode cm=NO_CONTROL) { setParameter("Kd", _kd, cm); }
    inline void setKd(double _kd, ControlMode cm=NO_CONTROL) { setParameter("Kd", Vector(1, &_kd), cm); kd=_kd;}
    inline void setKstic(double _ks){ k_stic=_ks; }
    inline void setKcoulomb(double _kc){ k_coul=_kc; }
    inline void setKcp(double _kcp){ k_cp=_kcp; }
    inline void setKcn(double _kcn){ k_cn=_kcn; }
    inline void setKdither(double _kdith){ k_dither=_kdith; }
    inline void setWdither(double _wdith){ w_dither=_wdith; }
    inline void setMd(double _v){ Md=_v; }
    inline void setM(double _v){ M=_v; }
    inline void setB(double _v){ B=_v; }
    inline void setK(double _v){ K=_v; }
    inline void setTaod(Vector _taod) {
        if(_taod.size() != taod.size()){
            stringstream ss;
            ss<<"Taod wrong size "<< _taod.size();
            throw runtime_error(ss.str().c_str());
        }
        taod = _taod;
    }
    inline void setTaod(double _taod) { taod = _taod; }
    inline void setPwmD(double _pwmD) { pwmD = _pwmD; }
    inline void setAlpha(double _alpha) {
        if(_alpha<0.0){
            stringstream ss;
            ss<<"Alpha not positive: "<< _alpha;
            throw runtime_error(ss.str().c_str());
        }
        alpha = _alpha;
        lpf_pwm->setCutFrequency(alpha);
    }
    inline void setAlphaStic(double _alpha) {
        if(_alpha<0.0){
            stringstream ss;
            ss<<"Alpha not positive: "<< _alpha;
            throw runtime_error(ss.str().c_str());
        }
        alphaStic = _alpha;
        lpf_torque->setCutFrequency(alphaStic);
    }
    inline void setKtao(double _ktao) {     k_tao = _ktao; }
    inline void setKbemf(double _kbemf) {   k_bemf = _kbemf; }
    inline void setEstWind(int _estWind) {   
        velEstWind = _estWind; 
        ctrlModeSem.wait();
        velEstimator = new AWLinEstimator(velEstWind, velEstThr); 
        ctrlModeSem.post();
    }
    inline void setEstThr(double _estThr) {     
        velEstThr = _estThr; 
        ctrlModeSem.wait();
        velEstimator = new AWLinEstimator(velEstWind, velEstThr); 
        ctrlModeSem.post();
    }
    inline void setTrajTime(double _tt){    trajGen->setT(_tt); }
    inline void setQd(double _qd){          qDes[0] = _qd; }

    inline void setCtrlMode(const ControlMode &cm){
        if(ctrlMode != cm){
            ctrlModeSem.wait();
            ctrlModeChanged = true;
            ctrlMode = cm;
            ctrlModeSem.post();
        }
    }

    // reset the integral error of the torque PID
    inline void resetTorquePid(){
        torquePid->reset(zeros(1));
    }

    inline void setSimMode(bool on){
        if(on)
            cmdMode = SIMULATION;
        else
            cmdMode = REAL;
    }

    inline void setJoint(unsigned int joint){
        if(ctrlMode != NO_CONTROL){
            throw runtime_error("It is not allowed to change joint while control loop is active.");
        }
        jointId = joint;
    }
    // GET METHODS
    inline ControlMode getCtrlMode(){ return ctrlMode; }
	inline ControlThreadStatus getThreadStatus(){ return thread_status; }
    inline BodyPart getBodyPart(){ return bodyPart; }
    inline double getKp(ControlMode cm=NO_CONTROL){ return kp; }
    inline double getKi(ControlMode cm=NO_CONTROL){ return ki; }
    inline double getKd(ControlMode cm=NO_CONTROL){ return kd; }
    inline double getKstic(){ return k_stic; }
    inline double getKcoulomb(){ return k_coul; }
    inline double getKcp(){ return k_cp; }
    inline double getKcn(){ return k_cn; }
    inline double getKdither(){ return k_dither; }
    inline double getWdither(){ return w_dither; }
    inline double getMd(){ return Md; }
    inline double getM(){ return M; }
    inline double getB(){ return B; }
    inline double getK(){ return K; }
    inline Vector getTao(){ return tao; }
    inline Vector getTaod(){ return taod; }
    inline double getPwmD(){ return pwmD; }
    inline double getAlpha(){ return alpha; }
    inline double getAlphaStic(){ return alphaStic; }
    inline int getJoint(){ return jointId; }
    inline int getEstWind(){ return velEstWind; }
    inline double getEstThr(){ return velEstThr; }
    inline double getKtao(){ return k_tao; }
    inline double getKbemf(){ return k_bemf; }
    inline double getTrajTime(){ return trajGen->getT(); }
    inline double getQd(){ return qDes[0]; }

};


}

} // end namespace

#endif
