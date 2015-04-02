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

#include "FingerTipPreprocessor.h"


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return 0;

    FingertipPreprocessorModule *module = new FingertipPreprocessorModule();

    module->runModule(argc,argv);
    
    delete module;

    return 0;
}


FingertipPreprocessorModule::FingertipPreprocessorModule(){
    mThread         = NULL;
    mModuleName[0]  = 0;
    mPeriod         = 0.02;
    bIsReady        = false;
}

bool FingertipPreprocessorModule::open(Searchable &s){
    if(bIsReady)
        return true;

    mPeriod = 0.01;

    char txtBuffer[256];         

    char name[256];

    if(s.check("name")){
        sprintf(txtBuffer,"/%s/rpc",s.find("name").toString().c_str());
        strcpy(name,s.find("name").toString().c_str());
    }else{
        sprintf(txtBuffer,"/%s/rpc","FingertipPreprocessor");
        strcpy(name,"FingertipPreprocessor");
    }
    mRpcPort.open(txtBuffer);
    mRpcPort.setStrict();
    attach(mRpcPort,true);

    if(s.check("period")){
        mPeriod = s.find("period").asInt();
    }

    if(s.check("period")){
        mPeriod = s.find("period").asInt();
    }
    char skinName[256];
    if(s.check("skinPort")){
        strcpy(skinName,s.find("skinPort").toString().c_str());
    }else{
        strcpy(skinName,"/skinPort");
    }


    mThread = new FingertipPreprocessorThread(int(floor(mPeriod*1000.0)),name, skinName);
    bIsReady = mThread->start();

    return bIsReady;        
}

bool FingertipPreprocessorModule::close(){

    mThread->stop();

    mRpcPort.close();

    return true;
}

bool FingertipPreprocessorModule::respond(const Bottle& command, Bottle& reply) {
    int index = 0;
    int cmdSize = command.size();

    bool retVal = true;
    
    if(cmdSize>0){
        switch(command.get(0).asVocab())  {
        default:
            retVal = false;
            break;            
        }         
        
    }else{
        retVal = false;
    }

    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }else{
        return Module::respond(command,reply);
    }
    
    return true;
}


double FingertipPreprocessorModule::getPeriod(){
    return 2.0;
}
int FingertipPreprocessorModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);                                    
}

bool   FingertipPreprocessorModule::updateModule() {
    return true;
}


FingertipPreprocessorThread::FingertipPreprocessorThread(int period,const char* moduleName, const char* skinPortName)
:RateThread(period)
{
    bIsReady        = false;

    mPeriod         = period;
    mTime           = 0.0;
    mPrevTime       =-1.0;    
    bIsReady        = true;

    strcpy(mSkinPortName,skinPortName);
    strcpy(mModuleName,moduleName);


    bIsReady = true;
}

FingertipPreprocessorThread::~FingertipPreprocessorThread(){
}

bool FingertipPreprocessorThread::threadInit(){
    if(bIsReady){
        char txt[256];
        sprintf(txt,"/%s%s",mModuleName, mSkinPortName);
        mSkinPort.open(txt);

        if(!Network::connect(mSkinPortName,txt)){
            cerr << "Warning: Unable to connect "<< mSkinPortName<<" to "<< txt<<endl;
            cerr << "         You will have to do this manually."<<endl;
        }

        sprintf(txt,"/%s/normal:o",mModuleName);
        mNormalPort.open(txt);

        sprintf(txt,"/%s/pressure:o",mModuleName);
        mPressurePort.open(txt);

        sprintf(txt,"/%s/sensor:o",mModuleName);
        mSensorPort.open(txt);
 
       // Hard coded fingerTip direction
        double vals[]={
          -1.00000,   0.00000,   0.00000,
          -0.39956,   0.00000,   0.91671,
          -0.39956,   0.00000,   0.91671,
          -1.00000,   0.00000,   0.00000,
          -0.78673,   0.60316,   0.13140,
          -0.30907,   0.47765,   0.82239,
          -0.00000,   1.00000,   0.00000,
           0.30907,   0.47765,   0.82239,
           0.78673,   0.60316,   0.13140,
           1.00000,   0.00000,   0.00000,
           0.39956,   0.00000,   0.91671,
           0.39956,   0.00000,   0.91671
        };
        mFingerTipPDir.Set(vals,12,3);

        bThresholdSetCounter = 20;
        for(int i=0;i<FINGERS_COUNT;i++){
            mSFingerThreshold[i].Resize(12);
            mSFingerThreshold[i].Zero();
        }
       return true;
    }else{
        return false;
    }
}

void FingertipPreprocessorThread::threadRelease(){
        mSkinPort.close();
        mNormalPort.close();
        mPressurePort.close();
        mSensorPort.close();
}

void FingertipPreprocessorThread::run(){
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


    ReadInput();

    ComputeOutputs();

    WriteOutput();


    mMutex.post();
}

void FingertipPreprocessorThread::ReadInput(){
    Vector *inputVec;
    inputVec = mSkinPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()!=SKIN_PART_SIZE){
            fprintf(stderr,"Warning: Bad vector size as input to fingertip:i port\n");
            mSkinInput.resize(1);
        }else{
            mSkinInput = *inputVec;
        }
    }
}

void FingertipPreprocessorThread::WriteOutput(){
    {
        Vector& vec = mNormalPort.prepare();
        vec.resize(FINGERS_COUNT*3);
        for(int i=0;i<FINGERS_COUNT;i++){
            vec[i*3+0] = mSFingerTipDir[i][0];
            vec[i*3+1] = mSFingerTipDir[i][1];
            vec[i*3+2] = mSFingerTipDir[i][2];
        }
        mNormalPort.write();
    }    

    {
        Vector& vec = mPressurePort.prepare();
        vec.resize(FINGERS_COUNT);
        for(int i=0;i<FINGERS_COUNT;i++){
            vec[i] = mSFingerTipPressure[i][0];
        }
        mPressurePort.write();
    }    

    {
        Vector& vec = mSensorPort.prepare();
        vec.resize(FINGERS_COUNT*12);
        for(int i=0;i<FINGERS_COUNT;i++){
            for(int j=0;j<12;j++){
                vec[i*12+j] = mSFingerTip[i][j];
            }
        }
        mSensorPort.write();
    }    
}

void FingertipPreprocessorThread::ComputeOutputs(){

    for(int i=0;i<FINGERS_COUNT;i++){
        mSFingerTip[i].Resize(12);
        mSFingerTipDir[i].Resize(3);
        mSFingerTipPressure[i].Resize(1);


        mSFingerTip[i].Zero();
        mSFingerTipDir[i].Zero();
        mSFingerTipPressure[i].Zero();
    }

    if(mSkinInput.size() != SKIN_PART_SIZE)
        return;
    
    if(bThresholdSetCounter>0){
        for(int i=0;i<FINGERS_COUNT;i++){
            int offset = (i==0?4:i-1)*12;
            for(int j=0;j<12;j++){
                double val                  = 255.0-mSkinInput[offset+j];
                if(val>mSFingerThreshold[i][j])
                    mSFingerThreshold[i][j] = val;
            }
        }
        bThresholdSetCounter--;
        if(bThresholdSetCounter==0){
            cerr << "Info: Calibration done"<<endl;
        }
    }
 

    for(int i=0;i<FINGERS_COUNT;i++){
        int offset = (i==0?4:i-1)*12;
        double sum = 0.0;
        for(int j=0;j<12;j++){
            double val                  = 255.0-mSkinInput[offset+j];
            val                        -= mSFingerThreshold[i][j];
            val                         = MAX(0,val);

            mSFingerTip[i][j]           = val;
            mSFingerTipPressure[i][0]  += val;
            mSFingerTipDir[i]          += mFingerTipPDir.GetRow(j) * val;
        }        
        if(mSFingerTipPressure[i][0]>1.0){
            mSFingerTipDir[i].Normalize();
        }else{
            mSFingerTipDir[i].Zero();
        }
        mSFingerTipPressure[i][0] *= (1.0/12.0);
    }

}
