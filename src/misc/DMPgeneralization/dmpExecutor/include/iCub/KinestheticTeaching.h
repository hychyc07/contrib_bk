/*
 * Copyright (C) 2013 Istituto Italiano di Tecnologia
 * Author:  Elena Ceseracciu
 * email: elena.ceseracciu@iit.it
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

// ADAPTATION FROM "actionsRenderingEngine" module in main iCub repository
#ifndef KINESTHETIC_TEACHING_THREAD_H
#define KINESTHETIC_TEACHING_THREAD_H


#include <string>
#include <iostream>
#include <yarp/os/RateThread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IImpedanceControl.h>
#include <yarp/dev/CartesianControl.h>

//#include <iCub/action/actionPrimitives.h>
#include <iCub/ctrl/pids.h>
#include <yarp/os/Semaphore.h>


YARP_DECLARE_DEVICES(icubmod)

class KinTeachingThread: public yarp::os::RateThread
{
    yarp::os::Semaphore mutex;
    

    double                      t0;
    double                      samplingRate;

    yarp::os::Bottle            actions;
    std::string                 actionName;

    iCub::ctrl::Integrator      *I;
    
    yarp::os::Property initializer;
    
    std::string robot;
    std::string name;

    bool status_impedance_on;
    int initial_context;
    
    yarp::dev::ICartesianControl *armCartCtrl;
    
    yarp::dev::IControlMode     *ctrl_mode_torso;
    yarp::dev::IControlMode     *ctrl_mode_arm;
    
    yarp::dev::IImpedanceControl           *ctrl_impedance_torso;
    yarp::dev::IImpedanceControl           *ctrl_impedance_arm;
     
    yarp::dev::PolyDriver armCartCtrlPD;
    yarp::dev::PolyDriver drv_arm;
    yarp::dev::PolyDriver drv_torso;
    
    
    bool setTorque(bool turn_on);
    bool setImpedance(bool turn_on);
public:
    KinTeachingThread(int period, const yarp::os::Property& properties); 
    
    ~KinTeachingThread();
    
    bool threadInit();
    void threadRelease();
    bool startRecordingAction(std::string actionName);
    bool stopRecodingAction(yarp::os::Bottle& bAction);

    void run();
    
};
     
#endif
     