/* 
* Copyright (C) 2011 MACSi Project
* Author: Serena Ivaldi
* email:  serena.ivaldi@isir.upmc.fr
* website: www.macsi.isir.upmc.fr
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/**
@ingroup icub_contrib_modules

\defgroup simWholeBodyTorqueObserver simWholeBodyTorqueObserver

This module, based on iCub wholeBodyTorqueObserver, is meant to provide
an emulation of the FT sensors measurements, along with corresponding
joint torques computed with iDyn library, to the iCub simulator.
Of course this module can be used only to help testing, it must not be considered as
a substitute of the real sensors and corresponding torque observer.
Using the autoconnect option, it automatically connects to the icub simulator, and
also connects to the vjoint ports used by the robotMotorGui to display joint torques.

\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 
- iKin library.
- iDyn library.  

\section parameters_sec Parameters

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
specified \e 10ms is assumed. 

--no_legs   
- this option disables the dynamics computation for the legs joints

--autoconnect
- this option automatically connects some ports

\section tested_os_sec Tested OS
Windows, Linux

\section example_sec Example
By launching the following command: 

\code 
simWholeBodyTorqueObserver --rate 10 --autoconnect 
\endcode 


*/ 

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <iomanip>
#include <string.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

#define MAX_JN 12
#define MAX_FILTER_ORDER 6
enum thread_status_enum {STATUS_OK=0, STATUS_DISCONNECTED}; 

// a filter used for the inertial sensor measurements
double lpf_ord1_3hz(double input, int j)
{ 
    if (j<0 || j>= MAX_JN)
    {
        cout<<"Received an invalid joint index to filter"<<endl;
        return 0;
    }

    static double xv[MAX_FILTER_ORDER][MAX_JN];
    static double yv[MAX_FILTER_ORDER][MAX_JN];
    xv[0][j] = xv[1][j] ; 
    xv[1][j] = input / 1.870043440e+01;
    yv[0][j] = yv[1][j] ; 
    yv[1][j] =   (xv[0][j]  + xv[1][j] ) + (  0.8930506128 * yv[0][j] );
    return (yv[1][j]);
}

// class virtualDynamics: class for reading from Vrow and providing FT on an output port
class virtualDynamics: public RateThread
{
private:

    bool       autoconnect;

    // Polydrivers
    PolyDriver *ddAL;
    PolyDriver *ddAR;
    PolyDriver *ddH;
    PolyDriver *ddLL;
    PolyDriver *ddLR;
    PolyDriver *ddT;

    // IEncoders
    IEncoders  *iencs_arm_left;
    IEncoders  *iencs_arm_right;
    IEncoders  *iencs_head;
    IEncoders  *iencs_leg_left;
    IEncoders  *iencs_leg_right;
    IEncoders  *iencs_torso;

    // Measures
    Vector *ft_arm_left;
    Vector *ft_arm_right;
    Vector *ft_leg_left;
    Vector *ft_leg_right;
    Vector *inertial;

    //input ports
    BufferedPort<Vector> *port_inertial_thread;

    //output ports
    // virtual FTS
    BufferedPort<Vector> *port_virtual_FTS_LA;
    BufferedPort<Vector> *port_virtual_FTS_RA;
    BufferedPort<Vector> *port_virtual_FTS_LL;
    BufferedPort<Vector> *port_virtual_FTS_RL;
    // torques
    BufferedPort<Bottle> *port_RATorques;
    BufferedPort<Bottle> *port_RLTorques;
    BufferedPort<Bottle> *port_RWTorques;
    BufferedPort<Bottle> *port_LATorques;
    BufferedPort<Bottle> *port_LLTorques;
    BufferedPort<Bottle> *port_LWTorques;
    BufferedPort<Bottle> *port_TOTorques;
    // external wrenches
    BufferedPort<Vector> *port_external_wrench_RA;
    BufferedPort<Vector> *port_external_wrench_LA;
    BufferedPort<Vector> *port_external_wrench_TO;

    bool first;
    thread_status_enum thread_status;

    // filters for velocity and acceleration
    AWLinEstimator  *InertialEst;
    AWLinEstimator  *linEstUp;
    AWQuadEstimator *quadEstUp;
    AWLinEstimator  *linEstLow;
    AWQuadEstimator *quadEstLow;

    int ctrlJnt;
    int allJnt;
    iCubWholeBody icub;
    iCubWholeBody icub_sens;

    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;

    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

    Vector q_head, dq_head, d2q_head;
    Vector q_larm, dq_larm, d2q_larm;
    Vector q_rarm, dq_rarm, d2q_rarm;
    Vector all_q_up, all_dq_up, all_d2q_up;

    Vector q_torso, dq_torso, d2q_torso;
    Vector q_lleg, dq_lleg, d2q_lleg;
    Vector q_rleg, dq_rleg, d2q_rleg;
    Vector all_q_low, all_dq_low, all_d2q_low;

    // inertial measures
    Vector w0,dw0,d2p0,inertial_measurements;
    // end effector vectors for initializating iDyn
    Vector Fend,Muend;
    // vectors for virtual FTS
    Vector Virtual_FTS_LL, Virtual_FTS_RL, Virtual_FTS_LA, Virtual_FTS_RA ;
    // others
    Vector F_LArm, F_RArm, F_iDyn_LArm, F_iDyn_RArm;
    Vector F_ext_left_arm, F_ext_right_arm, F_ext_torso;
    Vector F_LLeg, F_RLeg, F_iDyn_LLeg, F_iDyn_RLeg;
    Matrix F_sens_up, F_sens_low, F_ext_up, F_ext_low;
    
    // icub models
    Matrix FM_sens_up,FM_sens_low;

    // filtering joints velocities
    Vector evalVelUp(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEstUp->estimate(el);
    }
    Vector evalVelLow(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEstLow->estimate(el);
    }

    //filtering angular velocity for the head
    Vector eval_domega(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return InertialEst->estimate(el);
    }

    // filtering joints accelerations
    Vector evalAccUp(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEstUp->estimate(el);
    }
    Vector evalAccLow(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEstLow->estimate(el);
    }

    // init idyn body
    void init_upper()
    {
        //---------------------PARTS-------------------------//
        // Left_arm variables
        ft_arm_left = 0;
        allJnt = 0;
        int jnt=0;
        iencs_arm_left->getAxes(&jnt);
        encoders_arm_left.resize(jnt,0.0);
        F_LArm.resize(6,0.0);
        F_iDyn_LArm.resize(6,0.0);
        Virtual_FTS_LA.resize(6,0.0);
        q_larm.resize(7,0.0);
        dq_larm.resize(7,0.0);
        d2q_larm.resize(7,0.0);
        allJnt+=jnt;

        // Right_arm variables
        ft_arm_right = 0;
        jnt = 0;
        iencs_arm_right->getAxes(&jnt);
        encoders_arm_right.resize(jnt,0.0);
        q_rarm.resize(7,0.0);
        dq_rarm.resize(7,0.0);
        d2q_rarm.resize(7,0.0);
        F_RArm.resize(6,0.0);
        F_iDyn_RArm.resize(6,0.0);
        Virtual_FTS_RA.resize(6,0.0);
        allJnt+=jnt;

        // Head variables
        jnt = 0;
        iencs_head->getAxes(&jnt);
        encoders_head.resize(jnt,0.0);
        q_head.resize(3,0.0);
        dq_head.resize(3,0.0);
        d2q_head.resize(3,0.0);
        allJnt+=jnt;

        all_q_up.resize(allJnt,0.0);
        all_dq_up.resize(allJnt,0.0);
        all_d2q_up.resize(allJnt,0.0);
        F_sens_up.zero(); 
        FM_sens_up.resize(6,2); FM_sens_up.zero();
    }    
    void init_lower()
    {
        //---------------------PARTS-------------------------//
        // Left_arm variables
        ft_leg_left = 0;
        allJnt = 0;
        int jnt=0;
        if (iencs_leg_left) iencs_leg_left->getAxes(&jnt);
        else jnt = 6; //default value
        encoders_leg_left.resize(jnt,0.0);
        F_LLeg.resize(6,0.0);
        F_iDyn_LLeg.resize(6,0.0);
        Virtual_FTS_LL.resize(6,0.0);
        q_lleg.resize(6,0.0);
        dq_lleg.resize(6,0.0);
        d2q_lleg.resize(6,0.0);
        allJnt+=jnt;

        // Right_leg variables
        ft_leg_right = 0;
        jnt = 0;
        if (iencs_leg_right) iencs_leg_right->getAxes(&jnt);
        else jnt = 6; //default value
        encoders_leg_right.resize(jnt,0.0);
        q_rleg.resize(6,0.0);
        dq_rleg.resize(6,0.0);
        d2q_rleg.resize(6,0.0);
        F_RLeg.resize(6,0.0);
        F_iDyn_RLeg.resize(6,0.0);
        Virtual_FTS_RL.resize(6,0.0);
        allJnt+=jnt;

        // Head variables
        jnt = 0;
        iencs_torso->getAxes(&jnt);
        encoders_torso.resize(jnt,0.0);
        q_torso.resize(3,0.0);
        dq_torso.resize(3,0.0);
        d2q_torso.resize(3,0.0);
        allJnt+=jnt;

        all_q_low.resize(allJnt,0.0);
        all_dq_low.resize(allJnt,0.0);
        all_d2q_low.resize(allJnt,0.0);

        F_sens_low.zero();
        FM_sens_low.resize(6,2); FM_sens_low.zero();
    }

public:

    virtualDynamics(int _rate, PolyDriver *_ddAL, PolyDriver *_ddAR, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddLR, PolyDriver *_ddT, bool _autoconnect) : RateThread(_rate), ddAL(_ddAL), ddAR(_ddAR), ddH(_ddH), ddLL(_ddLL), ddLR(_ddLR), ddT(_ddT), autoconnect(_autoconnect)
    {        
        first = true;

        //--------------INTERFACE INITIALIZATION-------------//

        iencs_arm_left = 0;
        iencs_arm_right= 0;
        iencs_head     = 0;
        iencs_leg_left = 0;
        iencs_leg_right= 0;
        iencs_torso    = 0;

        //---------------------PORT--------------------------//

        // inertial sensor
        port_inertial_thread=new BufferedPort<Vector>;
        // virtual ft sensors
        port_virtual_FTS_LA=new BufferedPort<Vector>;
        port_virtual_FTS_RA=new BufferedPort<Vector>;
        port_virtual_FTS_LL=new BufferedPort<Vector>;
        port_virtual_FTS_RL=new BufferedPort<Vector>;
        // 
        port_RATorques = new BufferedPort<Bottle>;
        port_LATorques = new BufferedPort<Bottle>;
        port_RLTorques = new BufferedPort<Bottle>;
        port_LLTorques = new BufferedPort<Bottle>;
        port_RWTorques = new BufferedPort<Bottle>;
        port_LWTorques = new BufferedPort<Bottle>;
        port_TOTorques = new BufferedPort<Bottle>;
        port_external_wrench_RA = new BufferedPort<Vector>;  
        port_external_wrench_LA = new BufferedPort<Vector>;  
        port_external_wrench_TO = new BufferedPort<Vector>;  

        // the different part: in simWholeBody we don't have FT sensors, so we
        // create these ports
        port_virtual_FTS_LA->open("/icubSim/left_arm/analog:o");
        port_virtual_FTS_RA->open("/icubSim/right_arm/analog:o");
        port_virtual_FTS_LL->open("/icubSim/left_leg/analog:o");
        port_virtual_FTS_RL->open("/icubSim/right_leg/analog:o");

        port_inertial_thread->open("/simWholeBodyTorqueObserver/inertial:i");
        port_RATorques->open("/simWholeBodyTorqueObserver/right_arm/Torques:o");
        port_LATorques->open("/simWholeBodyTorqueObserver/left_arm/Torques:o");
        port_RLTorques->open("/simWholeBodyTorqueObserver/right_leg/Torques:o");
        port_LLTorques->open("/simWholeBodyTorqueObserver/left_leg/Torques:o");
        port_RWTorques->open("/simWholeBodyTorqueObserver/right_wrist/Torques:o");
        port_LWTorques->open("/simWholeBodyTorqueObserver/left_wrist/Torques:o");
        port_TOTorques->open("/simWholeBodyTorqueObserver/torso/Torques:o");
        port_external_wrench_RA->open("/simWholeBodyTorqueObserver/right_arm/endEffectorWrench:o"); 
        port_external_wrench_LA->open("/simWholeBodyTorqueObserver/left_arm/endEffectorWrench:o"); 
        port_external_wrench_TO->open("/simWholeBodyTorqueObserver/torso/Wrench:o");

        if (autoconnect)
        {
            Network::connect("/simWholeBodyTorqueObserver/filtered/inertial:o", "/simWholeBodyTorqueObserver/inertial:i","tcp",false);            
            Network::connect("/icubSim/inertial",                                "/simWholeBodyTorqueObserver/unfiltered/inertial:i","tcp",false);

            fprintf(stderr,"Connecting Torques:o to vjoints ports, if they exists... \n");
            
            if(!Network::connect("/simWholeBodyTorqueObserver/right_arm/Torques:o",    "/icubSim/joint_vsens/right_arm:i","udp",false))
                fprintf(stderr,"Could not connect right_arm to joint_vsens.. is it enabled on the iCub Simulator?\n");
            if(!Network::connect("/simWholeBodyTorqueObserver/left_arm/Torques:o",    "/icubSim/joint_vsens/left_arm:i","udp",false))
                fprintf(stderr,"Could not connect left_arm to joint_vsens.. is it enabled on the iCub Simulator?\n");
            if(!Network::connect("/simWholeBodyTorqueObserver/right_leg/Torques:o",    "/icubSim/joint_vsens/right_leg:i","udp",false))
                fprintf(stderr,"Could not connect right_leg to joint_vsens.. is it enabled on the iCub Simulator?\n");
            if(!Network::connect("/simWholeBodyTorqueObserver/left_leg/Torques:o",    "/icubSim/joint_vsens/left_leg:i","udp",false))
                fprintf(stderr,"Could not connect left_leg to joint_vsens.. is it enabled on the iCub Simulator?\n");

        }

        //---------------------DEVICES--------------------------//
        if (ddAL) ddAL->view(iencs_arm_left);
        if (ddAR) ddAR->view(iencs_arm_right);
        if (ddH)  ddH->view(iencs_head);
        if (ddLL) ddLL->view(iencs_leg_left);
        if (ddLR) ddLR->view(iencs_leg_right);
        if (ddT)  ddT->view(iencs_torso);

        linEstUp =new AWLinEstimator(16,1.0);
        quadEstUp=new AWQuadEstimator(25,1.0);
        linEstLow =new AWLinEstimator(16,1.0);
        quadEstLow=new AWQuadEstimator(25,1.0);
        InertialEst = new AWLinEstimator(16,1.0);

        //-----------parts INIT VARIABLES----------------//
        init_upper();
        init_lower();
        //-----------CARTESIAN INIT VARIABLES----------------//

        w0.resize(3,0.0);
        dw0.resize(3,0.0);
        d2p0.resize(3,0.0);
        Fend.resize(3,0.0);
        Muend.resize(3,0.0);
        F_ext_up.resize(6,3);
        F_ext_up = 0.0;
        F_ext_low.resize(6,3);
        F_ext_low = 0.0;
        inertial_measurements.resize(12);
        inertial_measurements.zero();
        F_ext_left_arm.resize(6,0.0);
        F_ext_right_arm.resize(6,0.0); 
    }

    bool threadInit()
    {   
        // in the simulator version, no caibration is required
        // since we don't have a FT sensor, we don't have an offset
        // which is due to the "real world" physical device 
        // anyway, it is good to put this blocking call to check it is
        // connected to the simulator

        //blocking call: waits for ports connection
        Vector *dummy = port_inertial_thread->read(true); 
        
        thread_status = STATUS_OK;
        printf ("iDyn thread started ..\n");
        return true;
    }

    inline thread_status_enum getThreadStatus() 
    {
        return thread_status;
    }

    void run()
    {   
        thread_status = STATUS_OK;

        // read joints
        if(readAndUpdateNOFTS(false) == false)
        {
            printf ("iDyn thread lost connection with the iCub simulator.\n");
            thread_status = STATUS_DISCONNECTED;
        }

        //estimate FTS
        computeFTSensors();
        F_LArm = Virtual_FTS_LA;
        F_RArm = Virtual_FTS_RA;
        F_LLeg = Virtual_FTS_LL;
        F_RLeg = Virtual_FTS_RL;

        // compute wholeBody
        Vector F_up(6);
        F_up=0.0;
        icub.upperTorso->setInertialMeasure(w0,dw0,d2p0);
        icub.upperTorso->setSensorMeasurement(F_RArm,F_LArm,F_up);
        icub.upperTorso->solveKinematics();
        icub.upperTorso->solveWrench();
        
        icub.lowerTorso->setInertialMeasure(icub.upperTorso->getTorsoAngVel(), 
            icub.upperTorso->getTorsoAngAcc(),
            icub.upperTorso->getTorsoLinAcc());

        Vector torso_F=icub.upperTorso->getTorsoForce();
        Vector torso_M=icub.upperTorso->getTorsoMoment();

        F_up[0]=torso_F[0];
        F_up[1]=torso_F[1];
        F_up[2]=torso_F[2];
        F_up[3]=torso_M[0];
        F_up[4]=torso_M[1];
        F_up[5]=torso_M[2];
        F_ext_torso=F_up;

        //fprintf (stderr,"FUP: %s %s \n",torso_F.toString().c_str(),torso_M.toString().c_str());
        //F_up.zero(); //comment this line to enable torso

        icub.lowerTorso->setSensorMeasurement(F_RLeg,F_LLeg,F_up);
        icub.lowerTorso->solveKinematics();
        icub.lowerTorso->solveWrench();

        Vector LATorques = icub.upperTorso->getTorques("left_arm");
        Vector RATorques = icub.upperTorso->getTorques("right_arm");
        Vector HDTorques = icub.upperTorso->getTorques("head");

        Vector LLTorques = icub.lowerTorso->getTorques("left_leg");
        Vector RLTorques = icub.lowerTorso->getTorques("right_leg");
        Vector tmp       = icub.lowerTorso->getTorques("torso");
        Vector TOTorques(3);

        TOTorques[0] = tmp [2];
        TOTorques[1] = tmp [1];
        TOTorques[2] = tmp [0];

        writeTorque(RATorques, 1, port_RATorques); //arm
        writeTorque(LATorques, 1, port_LATorques); //arm
        writeTorque(TOTorques, 4, port_TOTorques); //torso

        if (ddLR) writeTorque(RLTorques, 2, port_RLTorques); //leg
        if (ddLL) writeTorque(LLTorques, 2, port_LLTorques); //leg
        writeTorque(RATorques, 3, port_RWTorques); //wrist
        writeTorque(LATorques, 3, port_LWTorques); //wrist

        F_ext_left_arm=icub.upperTorso->left->getForceMomentEndEff();//-icub_sens.upperTorso->left->getForceMomentEndEff();
        F_ext_right_arm=icub.upperTorso->right->getForceMomentEndEff();//-icub_sens.upperTorso->right->getForceMomentEndEff();

        port_external_wrench_TO->prepare() = F_up;
        port_external_wrench_RA->prepare() = F_ext_right_arm;
        port_external_wrench_LA->prepare() = F_ext_left_arm;
        port_external_wrench_RA->write();
        port_external_wrench_LA->write();
        port_external_wrench_TO->write();
            
    }

    void threadRelease()
    {
        fprintf(stderr, "Closing the linear estimator\n");
        if(linEstUp)
        {
            delete linEstUp;
            linEstUp = 0;
        }
        if(linEstLow)
        {
            delete linEstLow;
            linEstLow = 0;
        }
        fprintf(stderr, "Closing the quadratic estimator\n");
        if(quadEstUp)
        {
            delete quadEstUp;
            quadEstUp = 0;
        }
        if(quadEstLow)
        {
            delete quadEstLow;
            quadEstLow = 0;
        }
        fprintf(stderr, "Closing the inertial estimator\n");
        if(InertialEst)
        {
            delete InertialEst;
            InertialEst = 0;
        }

        fprintf(stderr, "Closing RATorques port\n");
        closePort(port_RATorques);
        fprintf(stderr, "Closing LATorques port\n");
        closePort(port_LATorques);
        fprintf(stderr, "Closing RLTorques port\n");
        closePort(port_RLTorques);
        fprintf(stderr, "Closing LLTorques port\n");
        closePort(port_LLTorques);
        fprintf(stderr, "Closing RWTorques port\n");
        closePort(port_RWTorques);
        fprintf(stderr, "Closing LWTorques port\n");
        closePort(port_LWTorques);
        fprintf(stderr, "Closing TOTorques port\n");
        closePort(port_TOTorques);
        fprintf(stderr, "Closing external_wrench_RA port\n");
        closePort(port_external_wrench_RA);
        fprintf(stderr, "Closing external_wrench_LA port\n");    
        closePort(port_external_wrench_LA);
        fprintf(stderr, "Closing external_wrench_TO port\n");    
        closePort(port_external_wrench_TO);

        fprintf(stderr, "Closing inertial port\n");
        closePort(port_inertial_thread);
        fprintf(stderr, "Closing ft_arm_right port\n");
        closePort(port_virtual_FTS_RA);
        fprintf(stderr, "Closing ft_arm_left port\n");
        closePort(port_virtual_FTS_LA);
        fprintf(stderr, "Closing ft_leg_right port\n");
        closePort(port_virtual_FTS_RL);
        fprintf(stderr, "Closing ft_leg_left port\n");
        closePort(port_virtual_FTS_LL);      
    }   

    void closePort(Contactable *_port)
    {
        if (_port)
        {
            _port->interrupt();
            _port->close();

            delete _port;
            _port = 0;
        }
    }

    void writeTorque(Vector values, int address, BufferedPort<Bottle> *port)
    {
        Bottle a;
        a.addInt(address);
        for(int i=0;i<values.length();i++)
            a.addDouble(values(i));
        port->prepare() = a;
        port->write();
    }

    void writeVirtualFTS(Vector values, BufferedPort<Vector> *port)
    {
        port->prepare() = values;
        port->write();
    }

    void computeFTSensors()
    {

        Virtual_FTS_LA.zero();
        Virtual_FTS_RA.zero();
        Virtual_FTS_LL.zero();
        Virtual_FTS_RL.zero();

        //read joints and ft sensor
        readAndUpdateNOFTS(false,true);

        icub_sens.upperTorso->setInertialMeasure(w0,dw0,d2p0);
        Matrix F_sensor_up = icub_sens.upperTorso->estimateSensorsWrench(F_ext_up,false);
        icub_sens.lowerTorso->setInertialMeasure(icub_sens.upperTorso->getTorsoAngVel(),icub_sens.upperTorso->getTorsoAngAcc(),icub_sens.upperTorso->getTorsoLinAcc());
        Matrix F_sensor_low = icub_sens.lowerTorso->estimateSensorsWrench(F_ext_low,false);

        Virtual_FTS_LA = -1.0 * F_sensor_up.getCol(1);
        Virtual_FTS_RA = -1.0 * F_sensor_up.getCol(0);
        Virtual_FTS_LL = -1.0 * F_sensor_low.getCol(1);
        Virtual_FTS_RL = -1.0 * F_sensor_low.getCol(0);

        // now write the estimated sensors measurements to their dummy ports
        writeVirtualFTS(Virtual_FTS_LA,port_virtual_FTS_LA);
        writeVirtualFTS(Virtual_FTS_RA,port_virtual_FTS_RA);
        writeVirtualFTS(Virtual_FTS_LL,port_virtual_FTS_LL);
        writeVirtualFTS(Virtual_FTS_RL,port_virtual_FTS_RL);

    }

    bool readAndUpdateNOFTS(bool waitMeasure=false, bool _init=false)
    {
        bool b = true;

        //inertial sensor
        if (waitMeasure) fprintf(stderr,"Trying to connect to intertial sensor...");
        inertial = port_inertial_thread->read(waitMeasure);
        if (waitMeasure) fprintf(stderr,"done. \n");

        int sz = 0;
        if(inertial!=0)
        {
            sz = inertial->length();
            inertial_measurements.resize(sz) ;
            inertial_measurements= *inertial;
#ifdef DEBUG_FIXED_INERTIAL
            inertial_measurements[0] = 0;
            inertial_measurements[1] = 0;
            inertial_measurements[2] = 9.81;
            inertial_measurements[3] = 0;
            inertial_measurements[4] = 0;
            inertial_measurements[5] = 0;
#endif
            d2p0[0] = inertial_measurements[0];
            d2p0[1] = inertial_measurements[1];
            d2p0[2] = inertial_measurements[2];
            w0 [0] = inertial_measurements[3];
            w0 [1] = inertial_measurements[4];
            w0 [2] = inertial_measurements[5];
            dw0 = eval_domega(w0);
        }

        b &= getUpperEncodersSpeedAndAcceleration();
        setUpperMeasure(_init);
        b &= getLowerEncodersSpeedAndAcceleration();
        setLowerMeasure(_init);
        return b;
    }

    bool getLowerEncodersSpeedAndAcceleration()
    {
        bool b = true;
        if (iencs_leg_left)
        {b &= iencs_leg_left->getEncoders(encoders_leg_left.data());}
        else
        {encoders_leg_left.zero();}

        if (iencs_leg_right)
        {b &= iencs_leg_right->getEncoders(encoders_leg_right.data());}
        else
        {encoders_leg_right.zero();}

        b &= iencs_torso->getEncoders(encoders_torso.data());

        for (int i=0;i<q_torso.length();i++)
        {
            q_torso(i) = encoders_torso(2-i);
            all_q_low(i) = q_torso(i);
        }
        for (int i=0;i<q_lleg.length();i++)
        {
            q_lleg(i) = encoders_leg_left(i);
            all_q_low(q_torso.length()+i) = q_lleg(i);
        }
        for (int i=0;i<q_rleg.length();i++)
        {
            q_rleg(i) = encoders_leg_right(i);
            all_q_low(q_torso.length()+q_lleg.length()+i) = q_rleg(i);
        }
        all_dq_low = evalVelLow(all_q_low);
        all_d2q_low = evalAccLow(all_q_low);
        for (int i=0;i<q_torso.length();i++)
        {
            dq_torso(i) = all_dq_low(i);
            d2q_torso(i) = all_d2q_low(i);
        }
        for (int i=0;i<q_lleg.length();i++)
        {
            dq_lleg(i) = all_dq_low(i+q_torso.length());
            d2q_lleg(i) = all_d2q_low(i+q_torso.length());
        }
        for (int i=0;i<q_rleg.length();i++)
        {
            dq_rleg(i) = all_dq_low(i+q_torso.length()+q_lleg.length());
            d2q_rleg(i) = all_d2q_low(i+q_torso.length()+q_lleg.length());
        }

        return b;
    }

    bool getUpperEncodersSpeedAndAcceleration()
    {
        bool b = true;
        b &= iencs_arm_left->getEncoders(encoders_arm_left.data());
        b &= iencs_arm_right->getEncoders(encoders_arm_right.data());
        b &= iencs_head->getEncoders(encoders_head.data());        

        for (int i=0;i<q_head.length();i++)
        {
            q_head(i) = encoders_head(i);
            all_q_up(i) = q_head(i);
        }
        for (int i=0;i<q_larm.length();i++)
        {
            q_larm(i) = encoders_arm_left(i);
            all_q_up(q_head.length()+i) = q_larm(i);
        }
        for (int i=0;i<q_rarm.length();i++)
        {
            q_rarm(i) = encoders_arm_right(i);
            all_q_up(q_head.length()+q_larm.length()+i) = q_rarm(i);
        }
        all_dq_up = evalVelUp(all_q_up);
        all_d2q_up = evalAccUp(all_q_up);

        for (int i=0;i<q_head.length();i++)
        {
            dq_head(i) = all_dq_up(i);
            d2q_head(i) = all_d2q_up(i);
        }
        for (int i=0;i<q_larm.length();i++)
        {
            dq_larm(i) = all_dq_up(i+q_head.length());
            d2q_larm(i) = all_d2q_up(i+q_head.length());
        }
        for (int i=0;i<q_rarm.length();i++)
        {
            dq_rarm(i) = all_dq_up(i+q_head.length()+q_larm.length());
            d2q_rarm(i) = all_d2q_up(i+q_head.length()+q_larm.length());
        }

        return b;
    }

    void setLowerMeasure(bool _init=false)
    {
        if(!_init)
        {
            icub.lowerTorso->setAng("torso",CTRL_DEG2RAD * q_torso);
            icub.lowerTorso->setDAng("torso",CTRL_DEG2RAD * dq_torso);
            icub.lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * d2q_torso);

            icub.lowerTorso->setAng("left_leg",CTRL_DEG2RAD * q_lleg);
            icub.lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * dq_lleg);
            icub.lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * d2q_lleg);

            icub.lowerTorso->setAng("right_leg",CTRL_DEG2RAD * q_rleg);
            icub.lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * dq_rleg);
            icub.lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * d2q_rleg);
        }
        else
        {
            icub_sens.lowerTorso->setAng("torso",CTRL_DEG2RAD * q_torso);
            icub_sens.lowerTorso->setDAng("torso",CTRL_DEG2RAD * dq_torso);
            icub_sens.lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * d2q_torso);

            icub_sens.lowerTorso->setAng("left_leg",CTRL_DEG2RAD * q_lleg);
            icub_sens.lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * dq_lleg);
            icub_sens.lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * d2q_lleg);

            icub_sens.lowerTorso->setAng("right_leg",CTRL_DEG2RAD * q_rleg);
            icub_sens.lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * dq_rleg);
            icub_sens.lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * d2q_rleg);
        }
    }

    void setUpperMeasure(bool _init=false)
    {
        if(!_init)
        {
            icub.upperTorso->setAng("head",CTRL_DEG2RAD * q_head);
            icub.upperTorso->setAng("left_arm",CTRL_DEG2RAD * q_larm);
            icub.upperTorso->setAng("right_arm",CTRL_DEG2RAD * q_rarm);
            icub.upperTorso->setDAng("head",CTRL_DEG2RAD * dq_head);
            icub.upperTorso->setDAng("left_arm",CTRL_DEG2RAD * dq_larm);
            icub.upperTorso->setDAng("right_arm",CTRL_DEG2RAD * dq_rarm);
            icub.upperTorso->setD2Ang("head",CTRL_DEG2RAD * d2q_head);
            icub.upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * d2q_larm);
            icub.upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * d2q_rarm);
            icub.upperTorso->setInertialMeasure(w0,dw0,d2p0);
        }
        else
        {
            icub_sens.upperTorso->setAng("head",CTRL_DEG2RAD * q_head);
            icub_sens.upperTorso->setAng("left_arm",CTRL_DEG2RAD * q_larm);
            icub_sens.upperTorso->setAng("right_arm",CTRL_DEG2RAD * q_rarm);
            icub_sens.upperTorso->setDAng("head",CTRL_DEG2RAD * dq_head);
            icub_sens.upperTorso->setDAng("left_arm",CTRL_DEG2RAD * dq_larm);
            icub_sens.upperTorso->setDAng("right_arm",CTRL_DEG2RAD * dq_rarm);
            icub_sens.upperTorso->setD2Ang("head",CTRL_DEG2RAD * d2q_head);
            icub_sens.upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * d2q_larm);
            icub_sens.upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * d2q_rarm);
            icub_sens.upperTorso->setInertialMeasure(w0,dw0,d2p0);
        }
    }

    void setZeroJntAngVelAcc()
    {
        dq_head = 0.0;
        d2q_head = 0.0;
        dq_larm = 0.0;
        d2q_larm = 0.0;
        dq_rarm = 0.0;
        d2q_rarm = 0.0;

        dq_rleg=0.0;
        d2q_rleg=0.0;
        dq_lleg=0.0;
        d2q_lleg=0.0;
        dq_torso=0.0;
        d2q_torso=0.0;
    }

};


// a module used to filter inertial data
class dataFilter : public BufferedPort<Bottle>
{
private:
    BufferedPort<Vector> &port_filtered;
    Vector g;
    //Filter *filter;

    virtual void onRead(Bottle &b)
    {
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz = b.size();
        Vector x(sz);
        Vector inertial(sz);
        for(unsigned int i=0;i<sz;i++)
        {
            x[i]=b.get(i).asDouble();
            inertial(i)=lpf_ord1_3hz(x(i), i);
        }
        g[0] = inertial[3];
        g[1] = inertial[4];
        g[2] = inertial[5]; 
        g[3] = inertial[6];
        g[4] = inertial[7];
        g[5] = inertial[8];
        //g = (9.81/norm(g))*g;

        port_filtered.prepare() = g;
        port_filtered.setEnvelope(info);
        port_filtered.write();
    }
public:
    dataFilter(BufferedPort<Vector> &_port_filtered, ResourceFinder &rf):    
      port_filtered(_port_filtered)
      {
          /*Vector num(3);
          Vector den(3);
          num[0]=0.0030; num[1]=0.0059; num[2]=0.0030;
          den[0]=1.0000;den[1]=-1.8404;den[2]=0.8522;*/
          //          filter = new Filter(num,den,0.0);
          g.resize(6);
      }
};

// the module used to launch the thread
class simWholeBodyTorqueObserver: public RFModule
{
private:
    Property OptionsLeftArm;
    Property OptionsRightArm;
    Property OptionsHead;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    Property OptionsTorso;
    bool     legs_enabled;

    dataFilter *port_inertial_input;
    BufferedPort<Vector> port_filtered;    

    virtualDynamics *virtual_dyn;

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;

public:
    simWholeBodyTorqueObserver()
    {
        virtual_dyn=0;
        dd_left_arm=0;
        dd_right_arm=0;
        dd_head=0;
        dd_left_leg=0;
        dd_right_leg=0;
        dd_torso=0;
        legs_enabled = true;
    }

    virtual bool createDriver(PolyDriver *_dd)
    {
        if (!_dd || !(_dd->isValid()))
        {
            fprintf(stderr,"It is not possible to instantiate the device driver\nreturning...");
            return 0;
        }

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        port_filtered.open("/simWholeBodyTorqueObserver/filtered/inertial:o");
        port_inertial_input = new dataFilter(port_filtered, rf);
        port_inertial_input->useCallback();
        port_inertial_input->open("/simWholeBodyTorqueObserver/unfiltered/inertial:i");

        int rate = 10;

        //-----------------CHECK IF AUTOCONNECT IS ON-----------//
        bool autoconnect;
        if (rf.check("autoconnect"))
        {
            autoconnect = true;
            fprintf(stderr,"Autoconnect option found. \n");
        }
        else 
        {
                autoconnect = false;
                fprintf(stderr,"Autoconnect option not found, disabled. \n");
        }

        //------------------CHECK IF LEGS ARE ENABLED-----------//
        if (rf.check("no_legs"))
        {
            legs_enabled= false;
            fprintf(stderr,"'no_legs' option found. Legs will be disabled.\n");
        }

        //---------------------RATE-----------------------------//
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n", rate);
        }
        else
        {
            fprintf(stderr,"Could not find rate in the config file\n using 10ms as default");
            rate = 10;
        }

        Time::delay(5.0);

        //---------------------DEVICES--------------------------//

        OptionsHead.put("device","remote_controlboard");
        OptionsHead.put("local","/simWholeBodyTorqueObserver/head/client");
        OptionsHead.put("remote","/icubSim/head");

        dd_head = new PolyDriver(OptionsHead);
        if (!createDriver(dd_head))
        {
            fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
            return false;
        }
        else
            fprintf(stderr,"device driver created\n");

        OptionsLeftArm.put("device","remote_controlboard");
        OptionsLeftArm.put("local","/simWholeBodyTorqueObserver/left_arm/client");
        OptionsLeftArm.put("remote","/icubSim/left_arm");
        dd_left_arm = new PolyDriver(OptionsLeftArm);
        if (!createDriver(dd_left_arm))
        {
            fprintf(stderr,"ERROR: unable to create left arm device driver...quitting\n");
            return false;
        }

        OptionsRightArm.put("device","remote_controlboard");
        OptionsRightArm.put("local","/simWholeBodyTorqueObserver/right_arm/client");
        OptionsRightArm.put("remote","/icubSim/right_arm");
        dd_right_arm = new PolyDriver(OptionsRightArm);
        if (!createDriver(dd_right_arm))
        {
            fprintf(stderr,"ERROR: unable to create right arm device driver...quitting\n");
            return false;
        }

        if (legs_enabled)
        {
            OptionsLeftLeg.put("device","remote_controlboard");
            OptionsLeftLeg.put("local","/simWholeBodyTorqueObserver/left_leg/client");
            OptionsLeftLeg.put("remote","/icubSim/left_leg");
            dd_left_leg = new PolyDriver(OptionsLeftLeg);
            if (!createDriver(dd_left_leg))
            {
                fprintf(stderr,"ERROR: unable to create left leg device driver...quitting\n");
                return false;
            }

            OptionsRightLeg.put("device","remote_controlboard");
            OptionsRightLeg.put("local","/simWholeBodyTorqueObserver/right_leg/client");
            OptionsRightLeg.put("remote","/icubSim/right_leg");
            dd_right_leg = new PolyDriver(OptionsRightLeg);
            if (!createDriver(dd_right_leg))
            {
                fprintf(stderr,"ERROR: unable to create right leg device driver...quitting\n");
                return false;
            }
        }

        OptionsTorso.put("device","remote_controlboard");
        OptionsTorso.put("local","/simWholeBodyTorqueObserver/torso/client");
        OptionsTorso.put("remote","/icubSim/torso");

        dd_torso = new PolyDriver(OptionsTorso);
        if (!createDriver(dd_torso))
        {
            fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
            return false;
        }
        else
            fprintf(stderr,"device driver created\n");

        //--------------------------THREAD--------------------------
        virtual_dyn = new virtualDynamics(rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso, autoconnect);
        fprintf(stderr,"Force/Torque - iDyn thread istantiated ... \n");
        Time::delay(5.0);

        virtual_dyn->start();
        fprintf(stderr,"thread started\n");
        return true;
    }

    bool close()
    {
        fprintf(stderr,"closing... \n");     

        if (virtual_dyn)
        {
            fprintf(stderr,"Stopping the Force/Torque - iDyn thread...");     
            virtual_dyn->stop();
            fprintf(stderr,"iDyn thread stopped\n");     
            delete virtual_dyn;
            virtual_dyn=0;
        }

        fprintf(stderr,"interrupting the filtered port \n");     
        port_filtered.interrupt(); //CHECK THIS, SOMETIMES IT SEEMS TO BLOCK THE PORT
        fprintf(stderr,"closing the filtered port \n");     
        port_filtered.close();

        if (dd_left_arm)
        {
            fprintf(stderr,"Closing dd_left_arm \n");     
            dd_left_arm->close();
            delete dd_left_arm;
            dd_left_arm=0;
        }
        if (dd_right_arm)
        {
            fprintf(stderr,"Closing dd_right_arm \n");     
            dd_right_arm->close();
            delete dd_right_arm;
            dd_right_arm=0;
        }
        if (dd_head)
        {
            fprintf(stderr,"Closing dd_head \n");     
            dd_head->close();
            delete dd_head;
            dd_head=0;
        }

        if (dd_left_leg)
        {
            fprintf(stderr,"Closing dd_left_leg \n");     
            dd_left_leg->close();
            delete dd_left_leg;
            dd_left_leg=0;
        }
        if (dd_right_leg)
        {
            fprintf(stderr,"Closing dd_right_leg \n");     
            dd_right_leg->close();
            delete dd_right_leg;
            dd_right_leg=0;
        }
        if (dd_torso)
        {
            fprintf(stderr,"Closing dd_torso \n");     
            dd_torso->close();
            delete dd_torso;
            dd_torso=0;
        }

        if(port_inertial_input)
        {
            fprintf(stderr,"interrupting the inertial input port \n");     
            port_inertial_input->interrupt();
            port_inertial_input->close();
            delete port_inertial_input;
            port_inertial_input=0;
        }

        fprintf(stderr,"simWholeBodyTorqueObserver module was closed successfully! \n");     
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule() 
    {
        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            printf ("simWholeBodyTorqueObserver is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (virtual_dyn==0) 
            return false;
        thread_status_enum thread_status = virtual_dyn->getThreadStatus();
        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            printf ("simWholeBodyTorqueObserver module lost connection with iCub Simulator, now closing...\n");
            return false;
        }
        else
        {
            fprintf(stderr,"simWholeBodyTorqueObserver module was closed successfully! \n");    
            return true;
        }

    }
};






/////////////////////////////////////////////////////////////
//
//                        MAIN
//
/////////////////////////////////////////////////////////////
int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("simWholeBodyTorqueObserver/conf");
    rf.configure("MACSI_ROOT",argc,argv);

    printf("simWholeBodyTorqueObserver has the following context: %s \n", rf.getContextPath().c_str());
    Time::delay(2.0);


    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout<<"Error: could not connect to YARP network.. is the yarp server available?"<<endl;
        return -1;
    }

    simWholeBodyTorqueObserver obs;
    return obs.runModule(rf);
}

