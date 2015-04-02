// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

/**
 *
\defgroup icub_iCubMotionControllerExt iCubMotionControllerExt
@ingroup icub_lasaBox_modules

This module is an interface to the PID controller for realtime control of the whole icub through the access of a single desired joint position port.

\section intro_sec Description

This module mainly allows:

- To instantiate several body parts simultaneously and to control them through a single set of i/o ports
- To simultaneously send position orders to a PID controller (position should be smooth and streamed at a high rate, otherwise you may break your robot)
- To be protected against communication breakdown, i.e., the module assumes continuous streams of orders, so
  if some timeout occurs, the module stops the robot. This module should preferentially be run on the pc104.


Commands that can be typed in the terminal are:
\verbatim
run:    Enable the position controller
idle:   Suspend the controller
susp:   Suspend the controller
stop:   Suspend the controller
rest:   Put the robot back in the initial rest position, then idle mode
\endverbatim


\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--robot <string>:      the name ofthe robot to be controlled (icubSim by default)
--name <string>:       the module base name for port creation (iCubMotionControllerExt by default)
--period <double>:     control loop period (0.01 sec by default)
--part: <string list>: the parts do you want to instantiate (multiple instances is possible)
                        [right_arm | left_arm | right_leg | left_leg | head | torso]+
                       example: --part right_arm+left_arm+torso
                       fake part are also available. For instance, 
                       The following example: --part fake_right_arm+torso
                       will run a fake right_arm controller and the real torso controller
                       so that the input/output vector are of the right size.
\endverbatim

\section portsa_sec Ports Accessed

The module assumes a robot interface running (for instance either iCubInterface or the simulator). It accesses velocity and encoder ports.

\section portsc_sec Ports Created


Input ports:

    - /moduleName/rpc: single general command entry port to the module
    - /moduleName/targetPosition: input target position for all joints (The order follow that given in the parameters section)

Output ports:

    * /moduleName/position: output current position for all joints
    * /moduleName/velocity: output current velocity for all joints (the velocity feedback shoud be turned on on the interface configuration file)


\section in_files_sec Input Data Files

None

\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
iCubMotionControllerExt --robot icub --period 0.01 --part left_arm+right_arm
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/modules/iCubMotionControllerExt
**/


#ifndef VELOCITYCONTROLLERMODULE_H_
#define VELOCITYCONTROLLERMODULE_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#include "VelocityController.h"

class VelocityControllerThread;

class VelocityControllerModule: public Module
{
private:
    BufferedPort<Bottle>        mRpcPort;
      
    VelocityControllerThread   *mThread;
      
    vector<PolyDriver*>         mDrivers;
    vector<VelocityController*> mControllers;
      
    Property                    mParams;
      
    char                        mRobotName[256];
    char                        mModuleName[256];
    double                      mPeriod;
    
    bool                        bIsReady;

    bool                        mParts[8];
    
    bool                        bFakeDrivers;
public:
    VelocityControllerModule();

    virtual bool open(Searchable &s);
    virtual bool close();

    virtual bool respond(const Bottle& command, Bottle& reply);

    virtual double getPeriod();
    virtual bool   updateModule();
  
    virtual int runModule(int argc, char *argv[], bool skipFirst = true);
    
};



class VelocityControllerThread: public RateThread
{
private:
    vector<VelocityController*>    *mControllersPtr;
    int                             mPeriod;
    char                            mModuleName[256];

    BufferedPort<Vector>        mTargetPosPort;
    BufferedPort<Vector>        mTargetVelPort;
    BufferedPort<Vector>        mPosPort;
    BufferedPort<Vector>        mVelPort;

    double                      mTime;
    double                      mPrevTime;    

public:
            VelocityControllerThread(int period, const char* moduleName, vector<VelocityController*> *controllers);
    virtual ~VelocityControllerThread();

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

            Vector& DispatchVector(Vector& src, int cId, Vector &res);
};

#endif /*VELOCITYCONTROLLERMODULE_H_*/

