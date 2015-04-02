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

#ifndef YarpVectorBagTHREAD_H_
#define YarpVectorBagTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;

#include <vector>
#include <string>
#include "MathLib/MathLib.h"

using namespace MathLib;

class YarpVectorBagThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    double                  mStartTime;
    double                  mTime;
    double                  mPrevTime;
    double                  mPauseTime;

    bool                    bRecMode;
    bool                    bPlayMode;
    bool                    bLoopMode;
    bool                    bAllDoneMsg;    

    double                  mLastPrintTime;

    std::string                 mBagPath;
    std::vector<std::string>    mPortNames;
    std::vector<std::string>    mFilePortNames;
    std::vector<std::string>    mNewPortNames;
    std::vector<int>    mPortIndices;

    BufferedPort<yarp::sig::Vector>   *mInputPort;
    BufferedPort<yarp::sig::Vector>   *mOutputPort;
    
    int                         mNbPorts;
    std::vector<Matrix>         mBagsContent;
    std::vector<Vector>         mBagsTime;
    std::vector<int>            mBagsPos;
    std::vector<int>            mBagsLineSize;
            
    bool                        bDebug;

    bool                        bIsPaused;

public:
    YarpVectorBagThread(int period, const char* baseName, std::vector<std::string> &portNames, std::vector<std::string> &newNames, const char *bagPath, int mode);
    virtual ~YarpVectorBagThread();

            void SetDebug(bool mode);

            void SaveBag();
            bool LoadBag();

            void StartBag();
            void PauseBag();
            void ResumeBag();

    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
};

#endif

