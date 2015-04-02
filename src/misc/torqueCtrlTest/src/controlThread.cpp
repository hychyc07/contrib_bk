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

#include "iCub/torqueCtrlTest/controlThread.h"
#include "iCub/skinDynLib/common.h"

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <iCub/ctrl/math.h>
#include <iostream>
#include <math.h>

using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::torqueCtrlTest;

	
controlThread::controlThread(string _moduleName, string _robotName, int _period,
                             BodyPart _bodyPart, VerbosityLevel _verbose) 
: RateThread(_period), bodyPart(_bodyPart), verbose(_verbose)
{   
    name                = "torqueCtrlThread";
	ctrlMode            = NO_CONTROL;
    ctrlModeChanged     = true;
    cmdMode             = SIMULATION;
    interfaces          = new robot_interfaces(_moduleName.c_str(), _robotName.c_str());

    try
    {
        //---------------------PORTS-------------------------//	
        string slash = "/";
	    port_torques        = new BufferedPort<Vector>;
        port_monitor        = new BufferedPort<Vector>;
	    if(!port_torques->open((slash+_moduleName+"/torques:o").c_str()))
            throw runtime_error("It was not possible to open the torques:o port");    
	    if(!port_monitor->open((slash+_moduleName+"/monitor:o").c_str()))
            throw runtime_error("It was not possible to open the monitor:o port");

        durations.resize(12, 0.0);
        timestamps.resize(12, 0.0);

        pwm.resize(1);
        pwmD = 0.0;
        isMoving = false;
        isTorqueErrBig = desMotionDir = 0;
	    tao.resize(1);
        taod.resize(1);
        taoStic.resize(1);
        taoCoulomb = 0.0;
        q.resize(1);
        qDes.resize(1);
        qRef.resize(1);
        dqRef.resize(1);
        alpha = DEFAULT_ALPHA;
        jointId = DEFAULT_JOINT_ID;

        // parameters of the identified motor model
        double direction = (bodyPart==LEFT_ARM)?1.0:-1.0;
        k_tao = direction*DEFAULT_K_TAO;
        k_bemf = direction*DEFAULT_K_BEMF;
        k_stic = direction*DEFAULT_K_STIC;
        k_coul = direction*DEFAULT_K_COULOMB;
        k_cp = DEFAULT_K_CP;
        k_cn = DEFAULT_K_CN;
        // PID
        Vector Kp(1), Ki(1), Kd(1);
        Vector Wp(1), Wi(1), Wd(1);
        Vector N(1),  Tt(1);
        Matrix satLim(1,2);
        Kp=kp=DEFAULT_PID_KP[0];
        Ki=ki=DEFAULT_PID_KI[0];
        Kd=kd=DEFAULT_PID_KD[0];
        Wp=Wi=Wd=1.0;
        N=10.0;
        Tt=1.0;
        satLim(0,0)= DEFAULT_SAT_LIM_DOWN;
        satLim(0,1)= DEFAULT_SAT_LIM_UP;
        // filters
        velEstWind = DEFAULT_VEL_EST_WIND;
        velEstThr = DEFAULT_VEL_EST_THR;
        alphaStic = DEFAULT_ALPHA;
        // dither
        k_dither = DEFAULT_K_DITHER;
        w_dither = DEFAULT_W_DITHER;
        // impedance
        M = Md = DEFAULT_INERTIA;
        B = DEFAULT_DAMPING;
        K = DEFAULT_STIFFNESS;

        torquePid = new parallelPID(_period/1000.0,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);
        velEstimator = new AWLinEstimator(velEstWind, velEstThr);
        trajGen = new minJerkTrajGen(1, _period*1e-3, DEFAULT_TRAJ_TIME);
        lpf_torque = new FirstOrderLowPassFilter(alphaStic, _period*1e-3, zeros(1));
        lpf_pwm = new FirstOrderLowPassFilter(alpha, _period*1e-3, zeros(1));
    }
    catch(runtime_error){
        this->threadRelease();
        throw;
    }
}

bool controlThread::threadInit(){
    if(!interfaces->init())
    {
        thread_status = STATUS_DISCONNECTED;
        return false;
    }
    
    // estimate sensor noise
    printf("Start collecting torque sensor data...\n");
    int N = 300, i=0;
    Vector torques(N);
    for(i=0; i<N; i++)
        while(!interfaces->itrq[bodyPart]->getTorque(jointId, torques.data()+i))
            Time::delay(0.001);
    // compute mean and std dev
    double mean=0.0;
    torqueSensStdDev=0.0;
    for(i=0; i<N; i++) mean += torques[i];
    mean /= N;
    for(i=0; i<N; i++) torqueSensStdDev += pow(torques[i]-mean, 2);
    torqueSensStdDev = sqrt(torqueSensStdDev/(N-1));
    printf("Joint torque sensor standard deviation: %.4f Nm\n", torqueSensStdDev);
    printf("Considering a stiction max of 1 Nm, the suggested kp is %.2f\n", 1.0/(2.0*torqueSensStdDev) - 1.0);
    Time::delay(5.0);

    thread_status = STATUS_OK;
    return true;
}

void controlThread::threadRelease(){
    fprintf(stderr, "Setting position control mode.\n");
    interfaces->icmd[bodyPart]->setPositionMode(jointId);

	Time::delay(0.5);
    
    if(port_torques)    { port_torques->interrupt(); port_torques->close(); delete port_torques;  port_torques = 0;}
    fprintf(stderr,"Control thread release.\n");
    if(port_monitor)    { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
}

void controlThread::run(){
    updateDurations(0);
    if(timestamps[0]-timestamps[1]> 0.050*100 && thread_status!=STATUS_DISCONNECTED && getIterations()>0){
        printf("Too much time between updateRobotStatus() and sanityCheck(): %3.3f\n", timestamps[0]-timestamps[1]);
        for(unsigned int i=0;i<durations.size();i++)
            printf("duration %d: %3.3f\n", i, durations[i]);
        for(unsigned int i=0;i<timestamps.size();i++)
            printf("timestamp %d: %3.3f\n", i, timestamps[i]);
    }
    // check everything is working fine
    if(thread_status==STATUS_DISCONNECTED || sanityCheck()==false){
        thread_status = STATUS_DISCONNECTED;
        return;
    }    

    // read encoders, compute e.e. pose and force, jacobian, etc
    ctrlModeSem.wait();
    updateDurations(1);

    // CONTROL LOOP
    updateRobotStatus();
    updateDurations(2);

    ControlMode currentCtrlMode = ctrlMode;
    if(ctrlModeChanged){
        printf("Control mode just changed to %s.\n", ControlMode_desc[currentCtrlMode].c_str());
        // if the ctrl mode has changed, initialize the new ctrl mode
        initCtrlMode(currentCtrlMode);
        ctrlModeChanged = false;
    }
    ctrlModeSem.post();
    updateDurations(5);

    computePwm(currentCtrlMode);
    updateDurations(6);

    sendPwm(currentCtrlMode);
    updateDurations(8);

    // send monitoring data   
    prepareMonitorData(currentCtrlMode);
    port_monitor->prepare() = monitorData;
    updateDurations(9);

    port_monitor->write();
    updateDurations(10);
}
void controlThread::updateDurations(int index){
    double now = Time::now();
    timestamps[index] = now;
    durations[index] = now-timePre;
    timePre = now;
}

void controlThread::prepareMonitorData(const ControlMode &cm)
{
    monitorData.resize(6);
    monitorData[0] = pwmTimestamp;
    int i=1;
    for(int j=0; j<5; j++)
        monitorData[i++] = pwmAll[j];

    // *** PID monitoring ***
    //monitorData.resize(25);
    //monitorData[0]  = taod[0];
    //monitorData[1]  = tao[0];
    //monitorData[2]  = pwm(0);                   // pwm sent to the motor
    //monitorData[3]  = pwmRead;                  // pwm read from the motor
    //monitorData[4]  = q[0];
    //monitorData[5]  = dq;
    //monitorData[6]  = k_tao*tao[0] + k_bemf*dq;   // pwm predicted by motor model

    //// values of shoulder joints
    //int i=7;
    //for(int j=0; j<3; j++)
    //{
    //    monitorData[i++] = taoAll[j];
    //    monitorData[i++] = pwmAll[j];
    //    monitorData[i++] = qAll[j];
    //}

    //monitorData[16] = taoStic(0);           // estimated stiction torque
    //monitorData[17] = k_tao*taoStic(0);    // pwm to compensate stiction
    //monitorData[18] = qDes[0];
    //monitorData[19] = qRef[0];
    //monitorData[20] = dqRef[0];
    //monitorData[21] = ddqRef(0);
    //monitorData[22] = k_tao*taoCoulomb;
    //monitorData[23] = (pwmRead - k_bemf*dq)/k_tao - tao(0);   // friction observed
    //monitorData[24] = taoCoulomb;       // coulomb friction
}

void controlThread::updateRobotStatus()
{
    // *** JOINT TORQUES *** 
    double timestamp[NJ];
    if(!interfaces->itrq[bodyPart]->getTorques(taoAll))
        printf("Get joint torques failed!\n");
    else
        tao[0] = taoAll[jointId];
    Vector taoFilt = lpf_torque->filt(tao);

    if(!interfaces->readGravityTorques(bodyPart, taoG))
        //printf("Get gravity torques failed: %s\n", taoG.toString(2).c_str());
    
    // *** JOINT PWM *** 
    //if(!interfaces->iopl[bodyPart]->getOutputs(pwmAll))
    if(!interfaces->readPwm(bodyPart, pwmAll, &pwmTimestamp))
        printf("Get joint pwm failed!\n");
    else
        pwmRead = pwmAll[jointId];
    
    // *** JOINT ANGLE ***
    if(!interfaces->ienct[bodyPart]->getEncodersTimed(qAll, timestamp))
        printf("Get encoders failed!\n");
    else
        q[0] = qAll[jointId];

    // estimate joint velocity
    AWPolyElement el;
    el.data.resize(1,q[0]);
    el.time = timestamp[jointId];
    Vector vel = velEstimator->estimate(el);
    dq=vel[0];
    
    // estimate whether joint is moving
    if(fabs(dq)>5.0)
        isMoving = dq>0?1:-1;
    else if((isMoving==1 && dq<1.0) || (isMoving==-1 && dq>-1.0))
        isMoving = 0;

    // *** JOINT SPEED ***
    if(!interfaces->ienc[bodyPart]->getEncoderSpeeds(dqAll))
        printf("Get encoder speeds failed!\n");
    else
        dqFirmware = dqAll[jointId];

    // estimate torque due to stiction using previous PWM command
    if(isMoving==0)
    {
        double e_tao = taod(0)-taoFilt(0);
        double a = w_dither * k_dither/k_cp;
        double b = k_dither - a*k_cp;
        taoCoulomb = b*e_tao + (e_tao<0.0 ? -a*e_tao*e_tao : a*e_tao*e_tao);
        if(taoCoulomb>k_cp)         taoCoulomb = k_cp;
        else if(taoCoulomb<k_cn)    taoCoulomb = k_cn;

        /*taoCoulomb = 2.0/M_PI*atan(k_dither*e_tao);

        if(fabs(e_tao) > k_dither*torqueSensStdDev)
            isTorqueErrBig = taoCoulomb>0.0 ? 1 : -1;
        else if( (isTorqueErrBig==1 && taoCoulomb<0.0) || (isTorqueErrBig==-1 && taoCoulomb>0.0))
            isTorqueErrBig = 0;

        if(isTorqueErrBig == 0)
            taoCoulomb = 0.0;
        else
            taoCoulomb = e_tao>0.0 ? k_cp : k_cn;*/

        // if torque measure is uncertain, use gravity torque to estimate stiction direction
        /*if(isTorqueErrBig==0)
        {
            double e_tao2 = taod(0)-taoG(jointId);
            if(fabs(e_tao2)>1e-2)
                desMotionDir = e_tao2>0.0 ? 1 : -1;
            else if( (desMotionDir==1 && e_tao2<1e-3) || (desMotionDir==-1 && e_tao2>-1e-3))
                desMotionDir = 0;

            if(desMotionDir!=0)
                pwmCoulomb = desMotionDir>0.0 ? k_cp : k_cn;
        }*/

        // compensate only for the part of stiction that increases the tracking error    
        //if(e_tao*pwmSticComp < 0.0)     // if stiction compensation acts against the tracking
        //    if(fabs(k_tao*e_tao) < fabs(pwmSticComp))
        //        pwmSticComp += k_tao*e_tao; // remove a part of friction compensation
        //    else 
        //        pwmSticComp = 0.0;          // remove all friction compensation*/
    }
    else
        taoCoulomb = isMoving>0 ? k_cp : k_cn;

    taoStic(0) = 0.0;
    double taoMotor = (pwmRead - k_bemf*dq)/k_tao;
    double taoF_obs = taoMotor - taoFilt(0);
    double taoFC = k_coul*taoCoulomb;
    //if(!isMoving)
    if(taoFC*taoF_obs>=0.0 && fabs(taoF_obs)>fabs(taoFC))    // if observed friction > compensated friction => compensate more!
        taoStic(0) = taoF_obs - taoFC;

    // TRAJECTORY GENERATION
    trajGen->computeNextValues(qDes);
    qRef = trajGen->getPos();
    dqRef = trajGen->getVel();
    ddqRef = trajGen->getAcc();
}

void controlThread::initCtrlMode(const ControlMode &cm)
{
    switch(cm){
        case NO_CONTROL:
            updateDurations(3);
            interfaces->icmd[bodyPart]->setPositionMode(jointId);
            updateDurations(4);
            break;
        default:
            if(cmdMode == REAL){
                updateDurations(3);
                printf("setting open loop ctrl mode\n");
                interfaces->icmd[bodyPart]->setOpenLoopMode(jointId);
                updateDurations(4);
            }
    }
}
void controlThread::computePwm(const ControlMode &cm){    
    switch(cm)
    {
        case NO_CONTROL:
            break;
        case PID_CTRL:
            pwm(0) = torquePid->compute(taod, tao).operator ()(0);
            break;
        case OPEN_CTRL:
            pwm(0) = pwmD;
            break;
        case FEEDFORWARD_CTRL:
		    // it is stupid to compensate for stiction if the current torque error is zero!
            pwm(0) = k_tao*(taod(0) + k_stic*taoStic(0) + k_coul*taoCoulomb + torquePid->compute(taod, tao).operator ()(0)) + 0.9*k_bemf*dq;
            break;
        case POS_CTRL:
            pwm(0) = kp*(qRef(0)-q(0)) + kd*(dqRef(0)-dq);
            break;
        case DITHER_CTRL:
            pwm(0) = k_dither*sin(w_dither*Time::now());
            break;
        case IMPEDANCE_CTRL:
            taod(0) = taoG(jointId) + M*ddqRef(0) + B*(dqRef(0)-dq) + K*(qRef(0)-q(0));
            pwm(0) = k_tao*(taod(0) + k_stic*taoStic(0) + k_coul*taoCoulomb + torquePid->compute(taod,tao).operator ()(0)) + 0.9*k_bemf*dq;
            break;
        default:
            fprintf(stderr, "[computePwm] Unknown control mode: %d\n", cm);
    }
    // low pass filter and saturation
    if(pwm(0)>=DEFAULT_SAT_LIM_UP || pwm(0)<=DEFAULT_SAT_LIM_DOWN)
    {    
        printf("WARNING: pid is saturating!\n");
        pwm(0) = pwm(0)>0.0 ? DEFAULT_SAT_LIM_UP : DEFAULT_SAT_LIM_DOWN;
    }
    pwm = lpf_pwm->filt(pwm);
}
void controlThread::sendPwm(const ControlMode &cm){    
    if(cmdMode == REAL){
        //send the torque commands to the motors
        switch(cm)
        {
            case NO_CONTROL:
                break;
            default:
                interfaces->icmd[bodyPart]->setOpenLoopMode(jointId);
                interfaces->iopl[bodyPart]->setOutput(jointId, pwm(0));
        }       
    }

    // write torques on the output port
    port_torques->prepare() = pwm;
    updateDurations(7);
	port_torques->write();
}
bool controlThread::sanityCheck(){
    return true;
}
