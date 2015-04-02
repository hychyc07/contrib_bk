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

 
#include "YarpVectorSplitterThread.h"

#include <string.h>


YarpVectorSplitterThread::YarpVectorSplitterThread(int period, const char* baseName, std::vector<int> &nums)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    mNums = nums;
    mTotal = 0;
    for(int i=0;i<mNums.size();i++) mTotal += mNums[i];

    mOutputPort= NULL;
}

YarpVectorSplitterThread::~YarpVectorSplitterThread()
{}

bool YarpVectorSplitterThread::threadInit()
{
    char portName[256];
    snprintf(portName,256,"/%s/input",mBaseName);
    mInputPort.open(portName);

    mOutputPort = new BufferedPort<Vector>[mNums.size()];
    for(int i=0;i<mNums.size();i++){
        snprintf(portName,256,"/%s/output%02d",mBaseName,i);
        mOutputPort[i].open(portName);
    }

    mTime               = 0.0;
    mPrevTime           =-1.0;

    return true;
}

void YarpVectorSplitterThread::threadRelease()
{
    mInputPort.close();
    if(mOutputPort){
        for(int i=0;i<mNums.size();i++){
            mOutputPort[i].close();
        }
        delete [] mOutputPort;
    }
}

void YarpVectorSplitterThread::run()
{
    mMutex.wait();

    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        mMutex.post();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;


    // Read data from input port
    Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size() == mTotal){
            // Write data to output port
            int cnt = 0;
            for(int i=0;i<mNums.size();i++){
                Vector &outputVec = mOutputPort[i].prepare();
                outputVec.resize(mNums[i]);
                for(int j=0;j<mNums[i];j++){
                    outputVec[j] = (*inputVec)[cnt++];
                }
                mOutputPort[i].write();
            }
        }else{
            fprintf(stderr,"Bad input vector length: %d rather than %d\n",inputVec->size(),mTotal);
        }
    }

    mMutex.post();
}

