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

 
#include "DataStreamer.h"

DataStreamer::DataStreamer()
{
    mState              = DS_IDLE;
    mNextState          = DS_IDLE;
    bRecord             = false;
    bUseTime            = false;
    bLoop               = false;
    mCurrTime           = 0.0;
    mStartTime          = 0.0;
    mPauseTime          = 0.0;
    mStreamSize         = 0;
    mStreamMaxSize      = 8;
    mStreamLineSize     = 16;
    bOnce               = false;
    

    mStartTime = Time::now();
    
//    mPeriod = period;
}

DataStreamer::~DataStreamer()
{}



void DataStreamer::Update(double dt)
{    
    mCurrTime = Time::now();
    mCurrTime = mCurrTime - mStartTime;
    
    if(bUseTime)
        mInputVector(0) = mCurrTime;

    bool bWriteData = false;

    switch(mState){
    case DS_IDLE:
        mOutputVector.Zero();
        break;
        
    case DS_RUN:
        cout << "Time: "<<mCurrTime<<", line: "<< mCurrentLine <<"/"<<mStreamSize+1<<", max:"<<mStreamMaxSize<<" ";
        if(mCurrTime<0.0){
            cout << endl;
            break;
        }
        if(bRecord){
            mData.SetRow(mInputVector,mCurrentLine);
            cout <<"<Recording> ";
        }

        mData.GetRow(mCurrentLine,mOutputVector);
        if(!bRecord){
            cout <<"<Playing> ";
            if(bUseTime){
                while(mOutputVector(0) < mCurrTime){
                    mCurrentLine++;
                    if(mCurrentLine>=mStreamSize){
                        if(!bLoop){
                            mCurrentLine = mStreamSize-1;
                            mState = DS_PAUSE;
                        }else{
                            mCurrentLine    = 0;
                            mStartTime      = Time::now();
                            mCurrTime       = 0;
                        }
                        break;
                    }
                    mData.GetRow(mCurrentLine,mOutputVector);
                }
            }else{
                mCurrentLine++;
                if(mCurrentLine>=mStreamSize){
                    if(!bLoop){
                        mCurrentLine = mStreamSize-1;
                    }else{
                        mCurrentLine = 0;
                    }
                }
            }
        }else{
            
            mCurrentLine++;
            if(mCurrentLine>=mStreamSize)
                mStreamSize++;
        
            if(mCurrentLine>=mStreamMaxSize){
                if(!bLoop){
                    mCurrentLine = mStreamMaxSize-1;
                    mState = DS_PAUSE;                    
                }else{
                    mCurrentLine = 0;
                }
            }
        }
        if(bOnce){
            mState      = DS_PAUSE;
            mPauseTime  = mCurrTime;
            bOnce       = false;
        }

        cout << endl;
        break;

    case DS_PAUSE:
        mData.GetRow(mCurrentLine,mOutputVector);
        break;    
    }
}

void                DataStreamer::SetInput(const MathLib::Vector& input){
    if(bUseTime){
        mInputVector.Resize(input.Size()+1);
        mInputVector(0) = 0;
        mInputVector.SetSubVector(1,input);
    }else{
        mInputVector = input;
    }
}
MathLib::Vector&    DataStreamer::GetOutput(){
    if(bUseTime){
        mOutputVectorBuffer.Resize(mStreamLineSize-1);
        mOutputVector.GetSubVector(1,mStreamLineSize-1,mOutputVectorBuffer);
    }else{
        mOutputVectorBuffer = mOutputVector;
    }
    return mOutputVectorBuffer;
}


void    DataStreamer::Start(bool once){
    mState          = DS_RUN;
    mStartTime      = Time::now();
    mCurrentLine    = 0;
    bOnce       = once;

    mInputVector.Resize(mStreamLineSize);
    mInputVector.Zero();
}
void    DataStreamer::Stop(){
    if(mState!=DS_IDLE){
        mState = DS_IDLE;
    }
}
void    DataStreamer::Pause(){
    if(mState==DS_RUN){
        mState      = DS_PAUSE;
        mPauseTime  = Time::now();
    }
}
void    DataStreamer::Resume(bool once){
    if(mState==DS_PAUSE){
        mState      = DS_RUN;
        mStartTime  = mStartTime + (Time::now()-mPauseTime);
        bOnce       = once;
    }
}
void    DataStreamer::SetRecordMode(bool rec){
    if(rec && !bRecord){
        mCurrentLine = 0;
    }
    bRecord = rec;
}
void    DataStreamer::Save(const char* filename){
    Stop();
    MathLib::Matrix data;
    mData.GetRowSpace(0, mStreamSize,data);
    data.Save(filename);
}
void    DataStreamer::Load(const char* filename){
    Stop();
    mData.Load(filename);
    if(mData.RowSize()==0){
        cerr << "File "<<filename<<" not found or bad data..."<<endl;
    }else{
        mStreamSize = mData.RowSize();
        mStreamMaxSize = mData.RowSize();
        mData.Print();
        mStreamLineSize = mData.ColumnSize();
    }
}
void    DataStreamer::Clear(){
    Stop();
    mCurrentLine = 0;
    mStreamSize  = 0;
}
void    DataStreamer::SetStreamLineSize(int size){
    Stop();
    if(size<0) size = 0;
    mStreamLineSize = size;
    mData.Resize(mStreamMaxSize,mStreamLineSize,true);
}
void    DataStreamer::SetStreamMaxSize(int size){
    Stop();
    if(size<0) size = 0;
    mStreamMaxSize = size;
    mData.Resize(mStreamMaxSize,mStreamLineSize,true);
}
void    DataStreamer::SetUseTime(bool useTime){
    Stop();
    bUseTime = useTime;
    if(bUseTime){
        if(mStreamLineSize==0){
            mStreamLineSize = 1;
            mData.Resize(mStreamMaxSize,mStreamLineSize,true);
        }
    }
}
void    DataStreamer::SetLoop(bool loop){
    bLoop = loop;
}

