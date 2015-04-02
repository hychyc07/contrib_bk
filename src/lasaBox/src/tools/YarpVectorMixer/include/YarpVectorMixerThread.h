// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

#ifndef YarpVectorMixerTHREAD_H_
#define YarpVectorMixerTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#define MAX_INPUT_PORTS    64


class YarpVectorMixerThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    double                  mTime;
    double                  mPrevTime;
    double                  mStartTime;

    int                     mInputPortsCount;
    bool                    bFixPortsCount;

    bool                    bAuto;

    BufferedPort<Vector>    mInputPorts[MAX_INPUT_PORTS];
    Vector                  mInputVectors[MAX_INPUT_PORTS];
    int                     bGotData[MAX_INPUT_PORTS];

        
    BufferedPort<Vector>    mOutputPort;
    Vector                  mOutputVector;
    
    bool                    bShowTime;

    bool                    bStarted;
    
public:
    YarpVectorMixerThread(int period, const char* baseName);
    virtual ~YarpVectorMixerThread();


    void ShowTime(bool st);
    void SetPortNumber(int num);
    void AutoMode(bool mode);
    void Restart();
    void StartStop();

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
};

#endif

