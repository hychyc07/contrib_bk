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
\defgroup icub_FingertipPreprocessor FingertipPreprocessor
@ingroup icub_lasaBox_tools

This module, when connected to a hand skin part, process the fingertip-related data
and sends the average pressure value and estimated contact normal to an output port.

\section intro_sec Description

This module reads a skin part, and first compute a threshold value for each sensor
to cancel the baseline. The system simply gets the max value of 20 readings.
(The drift is not considered...)

Let's call this threshold for a sensor i by th_i.

The output values for each finger are:
- The 12 raw sensor value (inverted: s_i = MAX(0, (255-raw_sensor_i) - th_i))
- Average pressure (s_1 + ... + s_12)/12
- A rough estimate of the normal direction. Given a sensor 3d direction d_i, the normal is computed as: n = sum_i d_i * s_i and then normalized to unit norm.

Outputs are given in the followng order: thumb, index, middle, ring and pinky.


\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:        the module base name for port creation (FingertipPreprocessor by default)
--period <double>:      control loop period (0.01 sec by default)
--skinPort <string>:    the skin port to connect to
\endverbatim


\section Input ports:

- /moduleName/rpc   : command port (unused)
- /moduleName/skinPort:i    : input skin port

\section Output ports:

- /moduleName/pressure:o    : pressure output
- /moduleName/normal:o      : normal output
- /moduleName/sensor:o      : sensing units output (by blocks of 12)

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
FingertipPreprocessor --name RightHandPreprocessor --period 0.02 --skinPort /icub/skin/right_hand
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/FingertipPreprocessor
**/


#ifndef FingertipPreprocessorMODULE_H_
#define FingertipPreprocessorMODULE_H_

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


#define FINGERS_COUNT   5
#define SKIN_PART_SIZE  (12*16)

class FingertipPreprocessorThread;

class FingertipPreprocessorModule: public Module
{
private:
    BufferedPort<Bottle>            mRpcPort;
      
    FingertipPreprocessorThread    *mThread;
            
    Property                        mParams;
      
    char                            mModuleName[256];
    double                          mPeriod;
    
    bool                            bIsReady;

    char                            mPartName[256];

public:
    FingertipPreprocessorModule();

    virtual bool open(Searchable &s);
    virtual bool close();

    virtual bool respond(const Bottle& command, Bottle& reply);

    virtual double getPeriod();
    virtual bool   updateModule();
  
    virtual int runModule(int argc, char *argv[], bool skipFirst = true);
    
};



class FingertipPreprocessorThread: public RateThread
{
private:
    bool                        bIsReady;
    Semaphore                   mMutex;

    PolyDriver*                 mDriver;

    int                         mPeriod;
    char                        mModuleName[256];
    char                        mSkinPortName[256];

    double                      mTime;
    double                      mPrevTime;    

    BufferedPort<Vector>        mSkinPort;
    BufferedPort<Vector>        mNormalPort;
    BufferedPort<Vector>        mPressurePort;
    BufferedPort<Vector>        mSensorPort;

    Vector                      mSkinInput;

    int                         bThresholdSetCounter;
    // Threshold values
    MathLib::Vector             mSFingerThreshold[FINGERS_COUNT];

    // Raw values
    MathLib::Vector             mSFingerTip[FINGERS_COUNT];
    // Normal
    MathLib::Vector             mSFingerTipDir[FINGERS_COUNT];
    // Pressure
    MathLib::Vector             mSFingerTipPressure[FINGERS_COUNT];

    // Preferred directions
    MathLib::Matrix             mFingerTipPDir;


public:
            FingertipPreprocessorThread(int period,const char* moduleName, const char* skinPortName);
    virtual ~FingertipPreprocessorThread();

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

    void ComputeOutputs();
    void ReadInput();
    void WriteOutput();

};

#endif /*FingertipPreprocessorMODULE_H_*/

