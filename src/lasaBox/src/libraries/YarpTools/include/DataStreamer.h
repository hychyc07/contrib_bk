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

#ifndef DATASTREAMER_H_
#define DATASTREAMER_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include "MathLib/MathLib.h"

class DataStreamer
{
private:    
    //int         mPeriod;
    
    enum DataStreamerState{
        DS_IDLE = 0,
        DS_RUN,
        DS_PAUSE
    };
    
    DataStreamerState       mState;
    DataStreamerState       mNextState;

    bool                    bRecord;
    bool                    bUseTime;
    bool                    bLoop;
    bool                    bOnce;

    double                  mCurrTime;
    double                  mStartTime;
    double                  mPauseTime;

    
    MathLib::Matrix         mData;
    int                     mStreamSize;
    int                     mStreamMaxSize;
    int                     mStreamLineSize;
    int                     mCurrentLine;
    MathLib::Vector         mInputVector;
    MathLib::Vector         mOutputVector;
    MathLib::Vector         mOutputVectorBuffer;
    
    bool                    bWaitAtStart;
    
public:
            DataStreamer();
    virtual ~DataStreamer();

    void    SetStreamLineSize   (int size);
    void    SetStreamMaxSize    (int size);
    void    SetRecordMode       (bool rec = true);
    void    SetUseTime          (bool useTime = true);

    void    Start(bool once = false);
    void    Stop();
    void    Pause();
    void    Resume(bool once = false);
    void    Save(const char* filename);
    void    Load(const char* filename);
    void    Clear();
    void    SetLoop(bool loop);

    void    Update(double dt);
    
    
    void                SetInput(const MathLib::Vector& input);
    MathLib::Vector&    GetOutput();
};

#endif

