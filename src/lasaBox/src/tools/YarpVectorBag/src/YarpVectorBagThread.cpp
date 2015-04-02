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

 
#include "YarpVectorBagThread.h"

#include <string.h>
#include <fstream>
using namespace std;

#define BAG_BLOCK_SIZE      200
#define BAG_BLOCK_SAFETY    100

YarpVectorBagThread::YarpVectorBagThread(int period, const char* baseName, std::vector<std::string> &portNames,
                                         std::vector<std::string> &newNames, const char *bagPath, int mode)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);

    bRecMode    = (mode==0);
    bPlayMode   = !bRecMode;
    bLoopMode   = (mode==2);

    mBagPath    = bagPath;

    mInputPort  = NULL;
    mOutputPort = NULL;

    mNbPorts    = portNames.size();
    mPortNames  = portNames;

    mNewPortNames = newNames;
    
    for(int i=0;i<mNbPorts;i++) cout << mPortNames[i]<<endl;

    
    bIsPaused = true;
    bDebug = false;
}

YarpVectorBagThread::~YarpVectorBagThread()
{}

void YarpVectorBagThread::SetDebug(bool mode){
    bDebug = mode;
}
bool YarpVectorBagThread::threadInit()
{

    char portName[256];

    if(bRecMode){
        if(mNbPorts<=0)
            return false;
        mBagsContent.resize(mNbPorts);
        mBagsTime.resize(mNbPorts);
        
        mInputPort = new BufferedPort<yarp::sig::Vector>[mNbPorts];
        for(int i=0;i<mNbPorts;i++){
            snprintf(portName,256,"/%s%s",mBaseName,mPortNames[i].c_str());
            if(!mInputPort[i].open(portName)){
                fprintf(stderr,"Error: failed to open port %s\n",portName);
            }            
            if(!Network::connect(mPortNames[i].c_str(),portName)){
                fprintf(stderr,"Error: failed to connect %s to %s\n",mPortNames[i].c_str(),portName);
            }

            mBagsPos.push_back(0);
            mBagsLineSize.push_back(-1);
        }
    }

    if(bPlayMode){
        if(!LoadBag()){
            return false;
        }
    }


    mTime               = 0.0;
    mPauseTime          = 0.0;
    mPrevTime           =-1.0;
    mStartTime          =-1.0;

    bAllDoneMsg         = !bLoopMode;

    return true;
}

#define TEMPFOLDER "/tmp"

bool YarpVectorBagThread::LoadBag(){
    char portName[256];
    bool bOk = true;

    fprintf(stderr,"*** Loading bag ***\n");
    if(bOk){
        snprintf(portName,256,"rm -rf  %s/%s",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }
    if(bOk){
        snprintf(portName,256,"mkdir -p %s/%s",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }
    if(bOk){
        snprintf(portName,256,"mv %s %s/%s/bag.tar.gz",mBagPath.c_str(),TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }    
    if(bOk){
        snprintf(portName,256,"cd %s/%s ; tar -zxf bag.tar.gz",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }    
    if(bOk){
        snprintf(portName,256,"mv %s/%s/bag.tar.gz %s",TEMPFOLDER,mBaseName,mBagPath.c_str());
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }    

    snprintf(portName,256,"%s/%s/bagports.txt",TEMPFOLDER,mBaseName);
    ifstream fname;
    fname.open(portName);
    if(fname.is_open()){
        fprintf(stderr,"Opening port list saved in: \"%s\"\n",portName);
        fprintf(stderr,"  found: \n");
        while(!fname.eof()){
            string s;
            fname >> s;
            if(s.length()>0){
                mFilePortNames.push_back(s);
                cerr << "    "<< s << endl;
            }
        }
        fname.close();
    }else{
        fprintf(stderr,"Error while opening port list in file: \"%s\"\n",portName);        
        bOk = false;
    }        
    if(bOk){
        if(mNbPorts==0){
            mNbPorts = mFilePortNames.size();
            mPortNames = mFilePortNames;
            for(int i=0;i<mNbPorts;i++)
                mPortIndices.push_back(i);
        }else{
            std::vector<std::string>    portNames = mPortNames;

            mNbPorts = 0;
            mPortNames.clear();
            
            for(int i=0;i<int(portNames.size());i++){
                bool bFound = false;
                for(int j=0;j<int(mFilePortNames.size());j++){
                    if(mFilePortNames[j] == portNames[i]){
                        if(mNewPortNames.size()>i){
                            mPortNames.push_back(mNewPortNames[i]);
                            cerr << "Info: Renaming: port \""<< portNames[i] <<"\" to \""<< mNewPortNames[i]<<"\""<<endl;
                        }else{
                            mPortNames.push_back(portNames[i]);
                        }
                        mPortIndices.push_back(j);
                        mNbPorts++;
                        bFound = true;
                        break;
                    }
                }
                if(!bFound){
                    cerr << "Error: port \""<< portNames[i] <<"\"not found in bag... skipping"<<endl;
                }
            }    
        }
    }
    if(mNbPorts<=0){
        cerr << "Fatal error: no port to stream found. Exiting"<<endl;
    }
    if((!bOk)||(mNbPorts<=0)){
        return false;
    }
    
    mBagsContent.resize(mNbPorts);
    mBagsTime.resize(mNbPorts);

    mOutputPort = new BufferedPort<yarp::sig::Vector>[mNbPorts];
    for(int i=0;i<mNbPorts;i++){
        snprintf(portName,256,"%s",mPortNames[i].c_str());
        mOutputPort[i].open(portName);
    }
    for(int i=0;i<mNbPorts;i++){
        snprintf(portName,256,"%s/%s/bag_%04d.bin",TEMPFOLDER,mBaseName,mPortIndices[i]);
        Matrix A;
        if(A.LoadBinary(portName)){
            A.GetColumn(0,mBagsTime[i]);
            A.GetColumns(1,A.ColumnSize()-1,mBagsContent[i]);
            mBagsPos.push_back(0);
            mBagsLineSize.push_back(mBagsContent[i].ColumnSize());
            fprintf(stderr,"Bag %s sucessfully opened (%dx%d)\n",portName,mBagsContent[i].RowSize(),mBagsContent[i].ColumnSize());
        }else{
            fprintf(stderr,"Error: failed to open bag file %s\n",portName);
        }
    }
    
    return true;
    
}

void YarpVectorBagThread::SaveBag(){
    char portName[256];
    bool bOk = true;

    fprintf(stderr,"*** Saving bag ***\n");
    if(bOk){
        snprintf(portName,256,"rm -rf  %s/%s",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }

    if(bOk){
        snprintf(portName,256,"mkdir -p %s/%s",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }


    snprintf(portName,256,"%s/%s/bagports.txt",TEMPFOLDER,mBaseName);
    ofstream fname;
    fname.open(portName);
    if(fname.is_open()){
        for(int i=0;i<mNbPorts;i++){
            fname << mPortNames[i]<<endl;
        }
        fname.close();
        fprintf(stderr,"Port list sucessfully saved in: \"%s\"\n",portName);
    }else{
        fprintf(stderr,"Error while saving port list in file: \"%s\"\n",portName);        
        bOk = false;
    }        

    
    if(bOk){
        for(int i=0;i<mNbPorts;i++){
            snprintf(portName,256,"%s/%s/bag_%04d.bin",TEMPFOLDER,mBaseName,i);
            Matrix A(mBagsPos[i],1+mBagsContent[i].ColumnSize());
            A.SetColumn(mBagsTime[i],0);
            A.SetColumnSpace(mBagsContent[i],1);
            if(A.SaveBinary(portName)){
                fprintf(stderr,"Bag \"%s\" of port \"%s\" sucessfully saved\n",portName,mPortNames[i].c_str());
            }else{
                fprintf(stderr,"Error while saving bag \"%s\"\n",portName);
                bOk = false;
            }
        }
    }
            
    if(bOk){
        fprintf(stderr,"Archiving...\n");
        snprintf(portName,256,"cd %s/%s ; tar -cf bag.tar bag*",TEMPFOLDER,mBaseName,TEMPFOLDER,mBaseName,TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }
    if(bOk){
        snprintf(portName,256,"gzip %s/%s/bag.tar",TEMPFOLDER,mBaseName);
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }
    if(bOk){
        snprintf(portName,256,"mv %s/%s/bag.tar.gz %s",TEMPFOLDER,mBaseName,mBagPath.c_str());
        fprintf(stderr,"  cmd: %s\n",portName);
        if(system(portName)!=0){
            fprintf(stderr,"  Error...\n");            
            bOk = false;
        }
    }
    if(bOk){
        fprintf(stderr,"Bag \"%s\" sucessfully saved\n",mBagPath.c_str());
    }        
    

    fprintf(stderr,"*** Done ***\n");

}


void YarpVectorBagThread::threadRelease()
{
    char portName[256];

    if(bRecMode){
        SaveBag();
    }
    if(mInputPort){
        for(int i=0;i<mNbPorts;i++){
            mInputPort[i].close();
        }
        delete [] mInputPort;
    }
    if(mOutputPort){
        for(int i=0;i<mNbPorts;i++){
            mOutputPort[i].close();
        }
        delete [] mOutputPort;
    }
}
void YarpVectorBagThread::StartBag(){
    mMutex.wait();
    if(bRecMode){
        if(bIsPaused){
            mStartTime = -1.0;
            mPrevTime  = -1.0;
            mLastPrintTime = 0.0;
            bIsPaused  = false;
            cerr << "Info: starting recording..."<<endl;
        }else{
            cerr << "Warning: already recording..."<<endl;
        }
    }
    if(bPlayMode){
        if(bIsPaused){
            mStartTime = -1.0;
            mPrevTime  = -1.0;
            mLastPrintTime = 0.0;
            for(int i=0;i<mNbPorts;i++){
                mBagsPos[i]=0;
            }
            bIsPaused  = false;
            cerr << "Info: re-starting replay..."<<endl;
        }else{
            cerr << "Warning: already running"<<endl;
        }
    }


    mMutex.post();
}
void YarpVectorBagThread::PauseBag(){
    mMutex.wait();
    if(bRecMode){
        cerr << "Error: cannot pause in recording mode..."<<endl;
    }
    if(bPlayMode){
        if(!bIsPaused){
            mPauseTime = mTime;
            bIsPaused  = true;
            cerr << "Info: pausing replay"<<endl;
        }else{
            cerr << "Warning: already paused"<<endl;
        }
    }
    mMutex.post();
}
void YarpVectorBagThread::ResumeBag(){
    mMutex.wait();
    if(bRecMode){
        cerr << "Error: cannot resume in recording mode..."<<endl;
    }
    if(bPlayMode){
        if(bIsPaused){
            double now      = Time::now();
            mStartTime      = now-mPauseTime;
            mPrevTime       = now;
            mLastPrintTime  = mPauseTime;

            bIsPaused = false;
            cerr << "Info: resuming replay"<<endl;
        }else{
            cerr << "Warning: already running"<<endl;
        }
    }
    mMutex.post();
}


void YarpVectorBagThread::run()
{
    mMutex.wait();

    double now = Time::now();
    if(mStartTime<0.0){
        mStartTime = now;
        mLastPrintTime = 0;
    }
    if(mPrevTime<0.0){
        mPrevTime = 0.0;
    }else{
        mPrevTime = mTime;    
    }    
    if(bIsPaused){
        mMutex.post();
        return;
    }

    mTime       = now - mStartTime;
    double dt   = mTime - mPrevTime;

        

    if(mLastPrintTime<mTime-1.0){
        if(bDebug)
            fprintf(stderr,"Debug: Current time: %f\n",mTime);
        mLastPrintTime = mTime;
    }

    if(bRecMode){
        // Read data from input portnow
        for(int i=0;i<mNbPorts;i++){
            yarp::sig::Vector *inputVec = mInputPort[i].read(false);
            if(inputVec!=NULL){
                // First time
                if(mBagsLineSize[i]<=0){
                    mBagsLineSize[i] = (*inputVec).size();
                    mBagsTime[i].Resize(BAG_BLOCK_SIZE,false);mBagsTime[i].Zero();
                    mBagsContent[i].Resize(BAG_BLOCK_SIZE,mBagsLineSize[i],false);mBagsContent[i].Zero();
                }

                // Check line size
                if((*inputVec).size() == mBagsLineSize[i]){
                    mBagsTime[i](mBagsPos[i]) = mTime;
                    for(int j=0;j<inputVec->size();j++)
                        mBagsContent[i](mBagsPos[i],j) = (*inputVec)[j];
                    mBagsPos[i]++;
                    if(mBagsPos[i]+BAG_BLOCK_SAFETY>mBagsTime[i].Size()){
                        mBagsTime[i].Resize(mBagsTime[i].Size()+BAG_BLOCK_SIZE);
                        mBagsContent[i].Resize(mBagsContent[i].RowSize()+BAG_BLOCK_SIZE,mBagsContent[i].ColumnSize());
                        fprintf(stderr,"Debug: Resizing bag (%d) on input port %s\n",mBagsContent[i].RowSize(),mPortNames[i].c_str());  
                    }
                }else{
                    fprintf(stderr,"Error: Varying vector size on input port %s\n",mPortNames[i].c_str());
                }
            }
        }
    }


    if(bPlayMode){
        bool bAllDone = true;
        for(int i=0;i<mNbPorts;i++){
            //cout << mOutputVector(0) << " "<<mCurrTime<<endl;
            bool bSendOutput=false;
            if(mBagsTime[i](mBagsPos[i]) <= mTime){
                bSendOutput=true;
                while(mBagsTime[i](mBagsPos[i]) < mTime){
                    mBagsPos[i]++;
                    if(mBagsPos[i]>=mBagsTime[i].Size()){
                        bSendOutput=false;
                        break;
                    }
                }
            }else{
                if(mBagsPos[i]<mBagsTime[i].Size()){
                    bAllDone = false;
                }
            }
            if(bSendOutput){
                bAllDone = false;
                //fprintf(stderr,"Sending: %d/%d\n",mBagsPos[i],mBagsContent[i].RowSize());
                yarp::sig::Vector &outputVec = mOutputPort[i].prepare();
                int size = mBagsContent[i].ColumnSize();
                outputVec.resize(size);
                for(int j=0;j<size;j++)
                    outputVec[j] = mBagsContent[i].AtNoCheck(mBagsPos[i],j);
                mOutputPort[i].write();
            }
        }
        if(bAllDone && bAllDoneMsg){
            fprintf(stderr,"Debug: All bags are done\n");
            bAllDoneMsg = false;
        }
        if(bAllDone && bLoopMode){
            fprintf(stderr,"Debug: All bags are done. Looping\n");
            mStartTime = -1.0;        
            mPrevTime  = -1.0;        
            for(int i=0;i<mNbPorts;i++){
                mBagsPos[i]=0;
            }
        }
    }

    /*
    // Read data from input port
    Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){}

    // Write data to output port
    Vector &outputVec = mOutputPort.prepare();
    mOutputPort.write();
    */
    mMutex.post();
}

