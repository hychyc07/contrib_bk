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
\defgroup icub_iCubRecordAndReplay iCubRecordAndReplay
@ingroup icub_lasaBox_modules

This module can be used in cunjunction with the torque control modules
to perform record and replay with the torque-controller parts of the iCub such 
as the arms and legs.

\section intro_sec Description

This module allows some record and replay with torque-controllable parts of the iCub.

Important: For torque control, the wholeBodyDynamics app has to  
be up and running!  

Data can be saved and loaded. Path are relative to the working directory.

A data file is a matrix, where each column 
corresponds to a robot joint. Each row corresponds to a vector of 
joint values at a given time. Each row is read sequentially and the timestep is
defined by the running period of the module.



Rpc commands that can be run in the terminal are:
\verbatim
load <string>   : load a recorded data file to be replayed
save <string>   : save last recorded data
rec             : start recording
prep            : move to initial position
go              : execute the trajectory in PID 
                  mode if the start position is close to the current one
go force        : execute the trajectory in PID 
                  starts where ever the robot is located. This is dangerous. But can be useful...
stop            : stop playing / recording
gcmp            : switch to gravity compensation (torque) control mode
gcmp arm        : switch the arm in gravity compensation (torque) control mode
gcmp hand       : switch the hand in gravity compensation (torque) control mode
pos             : switch to position control mode
\endverbatim

An usual recording session looks like:
\verbatim
gcmp
rec
<move the robot around>
stop
save data.txt
\endverbatim

An usual replay session looks like:
\verbatim
pos
load data.txt
prep
<wait ~5 sec>
go
\endverbatim

\section dependencies_sec Dependencies

- YARP
- wholeBodyDynamics (needs to be running and connected)

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation (iCubRecordAndReplay by default)
--period <double>:  control loop period (0.01 sec by default)
--part <string>:    the icub part to be controlled (e.g. left_arm, right_leg, ...) (mandatory parameter)
--robot <string>:   the name of the robot (e.g. icub) (mandatory parameter)
\endverbatim


\section Input ports:

- /moduleName/partName/rpc

\section Output ports:

- /moduleName/partName/joints:o : streaming of the joints position

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
iCubRecordAndReplay --name RecordLeft --robot icub --part left_arm
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/modules/iCubRecordAndReplay
**/


#ifndef iCubRecordAndReplayMODULE_H_
#define iCubRecordAndReplayMODULE_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/all.h>

#include "MathLib/MathLib.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class iCubRecordAndReplayThread;

class iCubRecordAndReplayModule: public Module
{
private:
    BufferedPort<Bottle>        mRpcPort;
      
    iCubRecordAndReplayThread   *mThread;
      
    PolyDriver*                 mDriver;
      
    Property                    mParams;
      
    char                        mRobotName[256];
    char                        mModuleName[256];
    double                      mPeriod;
    
    bool                        bIsReady;

    char                        mPartName[256];

public:
    iCubRecordAndReplayModule();

    virtual bool open(Searchable &s);
    virtual bool close();

    virtual bool respond(const Bottle& command, Bottle& reply);

    virtual double getPeriod();
    virtual bool   updateModule();
  
    virtual int runModule(int argc, char *argv[], bool skipFirst = true);
    
};



class iCubRecordAndReplayThread: public RateThread
{
private:
    bool                        bIsReady;
    Semaphore                   mMutex;

    PolyDriver*                 mDriver;

    int                         mPeriod;
    char                        mModuleName[256];

    double                      mTime;
    double                      mPrevTime;    

    IPositionControl            *mPositionController; 
    IPidControl                 *mPIDController;
    IEncoders                   *mEncoders;
    IControlMode                *mControlModeController;
    IControlLimits              *mLimitsController;
    IAmplifierControl           *mAmplifierController;
    
    int                         mJointsSize;
    Vector                      mJointsPos;
    Vector                      mJointsLim[2];


    MathLib::Matrix             mData;
    int                         mPos;

    bool                        bReady;
    bool                        bPlaying;

    int                         bHasRecord;

    bool                        bPositionMode;

    BufferedPort<Vector>        mJointPort;

public:
    iCubRecordAndReplayThread(int period, PolyDriver* mDriver, const char * name);
    virtual ~iCubRecordAndReplayThread();

            void load(const char *name);
            void go(bool force = false);
            void move();

            void stop();

            void gravityComp(int part=0);
            void positionMode();

            void record();
            void save(const char *name);

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

};

#endif /*iCubRecordAndReplayMODULE_H_*/

