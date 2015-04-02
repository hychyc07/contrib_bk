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

 
#include "YarpVectorMixerThread.h"

#include <string.h>
#include <iostream>
using namespace std;


YarpVectorMixerThread::YarpVectorMixerThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    bShowTime = false;
    bStarted = false;
    bFixPortsCount = false;
    bAuto   = false;
    mInputPortsCount = 0;

}

YarpVectorMixerThread::~YarpVectorMixerThread()
{}


void YarpVectorMixerThread::ShowTime(bool st){
    bShowTime = st;
}
void YarpVectorMixerThread::SetPortNumber(int num){
    if((num>0)&&(num<=MAX_INPUT_PORTS)){
        bFixPortsCount = true;
        while(mInputPortsCount<num){
            char portName[256];
            snprintf(portName,256,"/%s/input%02d",mBaseName,mInputPortsCount);
            mInputPorts[mInputPortsCount].open(portName);
            mInputPortsCount++;
        }
    }
}
void YarpVectorMixerThread::AutoMode(bool mode){
    bAuto    = true;
    bStarted = true;
}


bool YarpVectorMixerThread::threadInit()
{


    
    char portName[256];
    if(!bFixPortsCount){
        snprintf(portName,256,"/%s/input%02d",mBaseName,mInputPortsCount);
        mInputPorts[mInputPortsCount].open(portName);
    }
    
    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    mTime               = 0.0;
    mPrevTime           =-1.0;
    mStartTime          =-1.0;
    
    for(int i=0;i<MAX_INPUT_PORTS;i++){
        bGotData[i]  = 0;
    }
    
    if(bAuto){
        cerr << "*** Streaming started ***"<<endl;
    }
    
    return true;
}

void YarpVectorMixerThread::threadRelease()
{
    for(int i=0;i<mInputPortsCount+1;i++)
        mInputPorts[i].close();
        
    mOutputPort.close();
}
void YarpVectorMixerThread::Restart(){
    mMutex.wait();
    mStartTime  = -1.0;
    mMutex.post();
}

void YarpVectorMixerThread::StartStop(){
    mMutex.wait();
    bStarted = !bStarted;
    if(bStarted){
        cerr << "*** Started ***"<<endl;
        mStartTime  = -1.0;        
    }else{
        cerr << "*** Stopped ***"<<endl;
    }
    mMutex.post();
}

void YarpVectorMixerThread::run()
{
    mMutex.wait();

    if(mPrevTime<0.0){
        mPrevTime   = Time::now();
        mMutex.post();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    if(mStartTime<0.0){
        mStartTime  = mTime;        
    }


    double dt   = mTime - mPrevTime;

    if(!bFixPortsCount){
        if(mInputPorts[mInputPortsCount].getInputCount()>0){
            if(mInputPortsCount<MAX_INPUT_PORTS-1){
                mInputPortsCount++;
                char portName[256];
                snprintf(portName,256,"/%s/input%02d",mBaseName,mInputPortsCount);
                mInputPorts[mInputPortsCount].open(portName);
            }
        }
    }        


    for(int i=0;i<mInputPortsCount;i++){
	    Vector *input = mInputPorts[i].read(false);
        if(input!=NULL){
            mInputVectors[i] = *input;
            bGotData[i] = 1;
        }
    }
    
    if(mInputPortsCount>0){
        int bDataReady = 0;
        for(int i=0;i<mInputPortsCount;i++){
            bDataReady += bGotData[i];
        }
        if(bDataReady==mInputPortsCount){
            int size = 0;
            for(int i=0;i<mInputPortsCount;i++){
                size += mInputVectors[i].size();
            }
            int cpos = 0;
            if(bShowTime){
                mOutputVector.resize(size+1);
                mOutputVector[0] = mTime-mStartTime;
                cpos ++;
            }else{
                mOutputVector.resize(size);            
            }
            for(int i=0;i<mInputPortsCount;i++){
                int size = mInputVectors[i].size();
                for(int j=0;j<size;j++){
                    mOutputVector[cpos+j] = mInputVectors[i][j];
                }
                cpos += size;
            }

            if(bStarted){
                //cout << mOutputVector.toString()<<endl;
                Vector &output = mOutputPort.prepare();
                output = mOutputVector;
                mOutputPort.write();
            }            
            for(int i=0;i<MAX_INPUT_PORTS;i++){
                bGotData[i]  = 0;
            }    
        }
    }

    mMutex.post();
}

