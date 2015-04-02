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


 
#include "HandSkinControllerThread.h"

#include <string.h>
#include <iostream>
#include <math.h>

using namespace std;

HandSkinControllerThread::HandSkinControllerThread(int period, const char* baseName, const char* part)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    strncpy(mPartName,part,256);
    strncpy(mBasePath,"./data/GraspAdaptation",256);
    bDebugMode = false;
}

HandSkinControllerThread::~HandSkinControllerThread()
{
    Stop();
}

bool HandSkinControllerThread::threadInit()
{

    // Create ports and connect some of them

    char portName[256],portName2[256];
    snprintf(portName,256,"/%s/fingertip:i",mBaseName);
    mFingerTipPort.open(portName);
    
    usleep(100*1000);
    
    snprintf(portName2,256,"/icub/skin/%s_arm",mPartName);
    cerr << "Probing skin port: "<<portName2<<endl;
    if(!yarp::os::Network::exists(portName2)){
        cerr << "Error port: "<< portName2<<" not found"<<endl;
        snprintf(portName2,256,"/icub/skin/%s_hand",mPartName);
        cerr << "Probing skin port: "<<portName2<<endl;
        if(!yarp::os::Network::exists(portName2)){
            cerr << "Error port: "<< portName2<<" not found"<<endl;
            cerr << "***********************************************"<<endl;
            cerr << "You will have to connect some skin to the port"<<endl;
            cerr << portName<< " yourself!!!!"<<endl;
            cerr << "***********************************************"<<endl;            
        }else{
            yarp::os::Network::connect(portName2,portName);
        }
    }else{
        yarp::os::Network::connect(portName2,portName);
    }
 
    snprintf(portName,256,"/%s/output:o",mBaseName);
    mOutputDataPort.open(portName);
    
    
    // Retreive the control interfaces
    bool bOk = true;
    if(mDriver){
        bOk &= mDriver->view(mEncoders);
        bOk &= mDriver->view(mLimitsController);
        bOk &= mDriver->view(mPIDController);
        bOk &= mDriver->view(mOpenLoopController);
        bOk &= mDriver->view(mControlModeController);
    }else{
        mEncoders           = NULL;
        mLimitsController   = NULL;
	    mPIDController      = NULL;
        mOpenLoopController = NULL;
        mControlModeController = NULL;
    }
    
    if(!bOk){
        fprintf(stderr,"Error: Problem getting drivers interfaces\n");
        return false;
    }

    // We should have a icub arm part, so we can fix these values
    mJointsSize         = 16;
    mJointsFingerOffset = 7;
    

    // Setting up some more variables
    
    for(int i=mJointsFingerOffset;i<mJointsSize;i++){
        mJointControlMode[i] = 0;
    }
    

    mJointsPos.resize(mJointsSize);
    mJointsTargetPos.resize(mJointsSize);
    mJointsTargetPredPos.resize(mJointsSize);
    mJointsTargetCmd.resize(mJointsSize);
    mJointsOutputPos.resize(mJointsSize);
    mJointsPosLimits[0].resize(mJointsSize);
    mJointsPosLimits[1].resize(mJointsSize);
    mJointsRange.resize(mJointsSize);
    mJointsRest.resize(mJointsSize);

    mJointsPos              = 0;
    mJointsTargetPos        = 0;
    mJointsTargetPredPos    = 0;
    mJointsTargetCmd        = 0;
    mJointsOutputPos        = 0;

    mJointsPosLimits[0] = 0;
    mJointsPosLimits[1] = 0;
    for(int i=0;i<mJointsSize;i++){
        mLimitsController->getLimits(i,mJointsPosLimits[0].data()+i,mJointsPosLimits[1].data()+i);
        // The joint range of distal joint has to be reduced artificially
        if((i==10)||(i==12)||(i==14)){
            mJointsPosLimits[1] = 80;
        }
        mJointsRange[i] = mJointsPosLimits[1][i] - mJointsPosLimits[0][i];
    }

    // Some predefine rest position for the hand
    mJointsRest[ 7] = 5;
    mJointsRest[ 8] = 80;
    mJointsRest[ 9] = 10;
    mJointsRest[10] = 10;
    mJointsRest[11] = 10;
    mJointsRest[12] = 10;
    mJointsRest[13] = 10;
    mJointsRest[14] = 10;
    mJointsRest[15] = 10;

    
    // Hard-coded PID values

    // PID direction
    mJointsPidFactor.resize(mJointsSize);
    mJointsPidFactor     = 0;    
    if(strcmp(mPartName,"left")==0){
        mJointsPidFactor[ 7] = -1.0;
        mJointsPidFactor[ 8] = -1.0;
        mJointsPidFactor[ 9] =  1.0;
        mJointsPidFactor[10] = -1.0;
        mJointsPidFactor[11] =  1.0;
        mJointsPidFactor[12] = -1.0;
        mJointsPidFactor[13] =  1.0;
        mJointsPidFactor[14] =  1.0;
        mJointsPidFactor[15] =  1.0;
    }else{
        mJointsPidFactor[ 7] =  1.0;
        mJointsPidFactor[ 8] = -1.0;
        mJointsPidFactor[ 9] =  1.0;
        mJointsPidFactor[10] = -1.0;
        mJointsPidFactor[11] =  1.0;
        mJointsPidFactor[12] =  1.0;
        mJointsPidFactor[13] =  1.0;
        mJointsPidFactor[14] = -1.0;
        mJointsPidFactor[15] =  1.0;
    }
    
    
    mJointsPidKpP.resize(mJointsSize);
    mJointsPidKdP.resize(mJointsSize);
    mJointsPidKdI.resize(mJointsSize);
    
    mJointsPidKpS.resize(mJointsSize);
    mJointsPidKdS.resize(mJointsSize);
    // PID KP, KD, KI for position control
    mJointsPidKpP=    60.0;
    mJointsPidKdP=     7.0;
    mJointsPidKdI=     2.0;
    

    // PID KP, KD, for pressure control
    mJointsPidKpS=    60.0;
    mJointsPidKdS=    10.0;

    // Thumb opposition (a bit less power)
    mJointsPidKpP[8]= 30.0;
    mJointsPidKdP[8]= 10.0;
    mJointsPidKdI[8]=  1.0;

    // Global gain
    mJointsPidGain =   1.0;
    
    mJointsPidGainGlobal = 1.0;
    
    bFirst              = true;

    mTime               = 0.0;
    mPrevTime           =-1.0;

    mCtrlMode           = CM_IDLE;

    // Fingertip data structure setup
	mFingerTip.resize(SKIN_FULL_SIZE);
    mFingerTipBaseline.resize(SKIN_FULL_SIZE);
    mFingerTipRaw.resize(SKIN_FULL_SIZE);
    mFingerTipBaseline      = 0;
    mFingerTipCalibCount    = 0;
    mFingerTipRaw           = 0;


    ResetSFingerData();



    bGotSkin        = false;
    bSendOutputData = false;

    bPidUsePressure = false;
        
    bFilterSkin     = true;
    mSkinFilterTau  = 0.08;
        
    bGradientAscent = true;
    
    
    bTestingOutput  = false;

    bBadSkin        = false;
    
    bNoMissingCheck = false;
    mBadFingerId    = 0;


    bAutoSaveTrainingData = false;
    mTrainingData;
    mTrainingDataPos = -1;;
    
    ResetSkin();

    return true;
}


void HandSkinControllerThread::LoadGMM(const char *name){
    // Load gmm file
    char txt[256];
    snprintf(txt,256,"%s/%s_p_gmm.txt",mBasePath,name);
    cout << "Loading GMM: "<<txt<<endl;
    if(mGMModel.loadParams(txt)){
        cout << "  Model is "<< mGMModel.mDim <<" dimensional and has " << mGMModel.mNbState <<" components"<<endl;
    }else{
        cout << "  Error while opening file: "<<txt<<endl;    
    }
}
void HandSkinControllerThread::LoadReplay(const char *name){
    // Load replay file
    char txt[256];
    snprintf(txt,256,"%s/%s_p.txt",mBasePath,name);
    cout << "Loading Replay: "<<txt<<endl;
    
    if(mReplayData.Load(txt)){
        cout << "ReplayData "<< txt<<" loaded: "<<mReplayData.RowSize()<<" "<<mReplayData.ColumnSize()<<endl;
    }else{
        cout << "  Error while opening file: "<<txt<<endl;    
    }

    mReplayDt        = 0.04;
    mReplayStartTime = 0.0;    
}

void HandSkinControllerThread::threadRelease()
{
    // Cleanup
    mJointsOutputPos = mJointsPos;
    SendPIDControl();

    mFingerTipPort.close();
    mOutputDataPort.close();
    
}

// Main loop...
void HandSkinControllerThread::run()
{
    mMutex.wait();

    // Some timing code
    if(mPrevTime<0.0){
        mPrevTime   = Time::now();
        mStartTime  = mPrevTime;
        mMutex.post();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;



    if(mDriver!=NULL){
        // Get joint position
        mEncoders->getEncoders(mJointsPos.data());
        // Read and process skin
        ReadFromPort();
    }

    // Initialization on the first pass
    if(bFirst){
        mJointsTargetPos     = mJointsPos;
        mJointsTargetPredPos = mJointsPos;
        bFirst = false;
    }


    // Read sensor data and prepare SFinger data structure
    PrepareSFingers();


    // By default
    bPidUsePressure = false;

    // Replay control block
    if((mCtrlMode == CM_REPLAY)||(mCtrlMode == CM_REPLAYSIM)){
        // We replay... so getting time and index of the sorce data array
        double  replayTime  = mTime - mReplayStartTime;
        int     replayId    = int(floor(replayTime/mReplayDt));
        if(replayId<0){
            replayId = 0;
        }else if(replayId>=mReplayData.RowSize()){
            replayId = mReplayData.RowSize()-1;
            bReplayDone = true;
        }

        // Set target position from the replay array
        mJointsTargetPos = mJointsRest; // Just copy default data
        for(int i=mJointsFingerOffset;i<mJointsSize;i++){
            mJointsTargetPos[i] = mReplayData(replayId,i-mJointsFingerOffset);
        }
            
        // Set target pressure from the replay array
        mSFingerDesTip[0][0] = mReplayData(replayId,9);
        mSFingerDesTip[1][0] = mReplayData(replayId,10);
        mSFingerDesTip[2][0] = mReplayData(replayId,11);
        
        // If no driver, fake the skin response as well
        // (REPLAYSIM is for simulation) not really useful finally
        if((mDriver==NULL)||(mCtrlMode == CM_REPLAYSIM)){
            for(int i=0;i<FINGERS_COUNT;i++){
                mSFingerTipDir[i].Zero();
                mSFingerTipDir[i](0) = mReplayData(replayId,18+i*3+0);
                mSFingerTipDir[i](1) = mReplayData(replayId,18+i*3+1);
                mSFingerTipDir[i](2) = mReplayData(replayId,18+i*3+2);
            }
        }

        // Are we done... say hi from time to time
        if(!bReplayDone){
            if(replayId+1==mReplayData.RowSize())
                cout << "Replaying done"<<endl;
            else if((replayId)%50==0)
                cout << "Replaying "<<replayId+1<<"/"<<mReplayData.RowSize()<<endl;
        }        
    }

    // Depending on the control mode: do the prediction
    if( (mCtrlMode == CM_RUN)||
        (mCtrlMode == CM_RECRUN)||
        (mCtrlMode == CM_REPLAYSIM)){
            PredictPoseFromAngle();
    }

    // Some default value
    bPidUseI = true;

    switch(mCtrlMode){
    case CM_IDLE:   // Stay where you are
        mJointsTargetPos = mJointsPos;
        SetPIDMode();
        break;
    case CM_REST:   // Go to rest position
        mJointsPidGain   = 1.0;  
        bPidUsePressure  = false;        
        mJointsOutputPos = mJointsRest;
        SendOpenLoopControl();
        break;
    case CM_RUN:    // Execute the task!
        mJointsPidGain   = 1.0;  
        bPidUsePressure  = true;        
        mJointsOutputPos = mJointsTargetPredPos;
        SendOpenLoopControl(&mSFingerDesCoupling);
        break;         
    case CM_RECRUN: // Execute the task, but with lower gains and no pid 
                    // (good for recording a new policy)
        mJointsPidGain   = 0.3;  
        bPidUseI         = false;
        bPidUsePressure  = true;        
        mJointsOutputPos = mJointsTargetPredPos;
        SendOpenLoopControl(&mSFingerDesCoupling);
        break;              
    case CM_REC:    // Just lower the gain to stay where your were
                    // (good for recording a new policy, too)
        mJointsPidGain   = 0.3;  
        bPidUsePressure  = false;        
        mJointsOutputPos = mJointsTargetPos;
        SendOpenLoopControl();
        break;
    case CM_REPLAY: // Start a replay
        mJointsPidGain   = 1.5;  
        bPidUsePressure  = true;        
        mJointsOutputPos = mJointsTargetPos;
        {MathLib::Matrix eye(10,10);
            eye.Identity();
            eye*= 1.0/(10.0*10.0);
        SendOpenLoopControl(&eye);
        }
        break;
    case CM_REPLAYSIM:  // Start a replay (in simulation!?!?, not very useful)
        mJointsPidGain   = 1.0;  
        bPidUsePressure  = true;        
        mJointsOutputPos = mJointsTargetPredPos;
        SendOpenLoopControl(&mSFingerDesCoupling);
        break;
    }
     
    // Send data
    SendOutputData();

    mMutex.post();
}

// Sets the hand in PID mode
void HandSkinControllerThread::SetPIDMode(){
    if(mPIDController){
        for(int i=mJointsFingerOffset;i<mJointsSize;i++){
            if(mJointControlMode[i]!=1){
                mControlModeController->setPositionMode(i);
                mJointControlMode[i] = 1;
                double val = TRUNC(mJointsPos[i],mJointsPosLimits[0][i],mJointsPosLimits[1][i]);
                mPIDController->setReference (i, val);
            }                    
        }
    }
}

// Sends PID signals
void HandSkinControllerThread::SendPIDControl(){
    if(mPIDController){
        for(int i=mJointsFingerOffset;i<mJointsSize;i++){
            if(mJointControlMode[i]!=1){
                mControlModeController->setPositionMode(i);
                mJointControlMode[i] = 1;
            }        
            double val = TRUNC(mJointsOutputPos[i],mJointsPosLimits[0][i],mJointsPosLimits[1][i]);
            mPIDController->setReference (i, val);
            
        }
    }
}


// The open-loop controller :)
void  HandSkinControllerThread::SendOpenLoopControl(MathLib::Matrix * weights){
    bool bdm = bDebugMode;

    if(bDebugMode) cout << "Control loop ********* "<<endl;

    if(bDebugMode){
        cout << "Input Pressure:  "<< mSFingerTip[0][0]<<" "<<mSFingerTip[1][0]<<" "<<mSFingerTip[2][0]<<endl;
    }

    // Temporary error signals for computing the final control. 
    // For each finger: 2 position error signal + 1 pressure error signal
    // And a position error signal for the thunb opposition
    MathLib::Vector errorSignal(FINGERS_COUNT*2+1+FINGERS_COUNT*1);  // p signal
    MathLib::Vector dErrorSignal(FINGERS_COUNT*2+1+FINGERS_COUNT*1); // d signal
    MathLib::Vector iErrorSignal(FINGERS_COUNT*2+1+FINGERS_COUNT*1); // i signal
    
    // Another backup temporary vector
    MathLib::Vector errorSignalClean(FINGERS_COUNT*2+1+FINGERS_COUNT*1);

    static MathLib::Vector transErrorSignalClean(FINGERS_COUNT*2+1+FINGERS_COUNT*1);
    MathLib::Vector transErrorSignalClean0(FINGERS_COUNT*2+1+FINGERS_COUNT*1);

    MathLib::Vector tmpErrorSignalClean(FINGERS_COUNT*2+1+FINGERS_COUNT*1);

    mJointsTargetCmd = 0;
    

    // Lets display some values...    
    if(bDebugMode){
        MathLib::Vector target(7+3);
        MathLib::Vector current(7+3);
        for(int i=0;i<7;i++){
            target(i)  = mJointsOutputPos[8+i];
            current(i) = mJointsPos[8+i];
        }
        for(int i=0;i<3;i++){
            target(7+i)   = mSFingerDesTip[i][0];
            current(7+i)  = mSFingerTip[i][0];
        }
        cout << "Target:  "<< target<<endl;
        cout << "Current: "<< current<<endl;
    }


    // Computing errors for each finger
    for(int i=0;i<FINGERS_COUNT;i++){

        // Position error
        double posErr0              = mJointsOutputPos[9+i*2 +0]  - mJointsPos[9+i*2 +0];
        double posErr1              = mJointsOutputPos[9+i*2 +1]  - mJointsPos[9+i*2 +1];

        // a copy
        errorSignalClean(1+i*2 + 0) = posErr0;
        errorSignalClean(1+i*2 + 1) = posErr1;

        // If pressure control is required
        if(bPidUsePressure){
            // Presure error
            double tipErr   = mSFingerDesTip[i][0]    - mSFingerTip[i][0];
            // a copy
            errorSignalClean(1+FINGERS_COUNT*2+i)   = tipErr;
        }
    }

    // Computing errors for thumb opposition
    double posErr1      = mJointsOutputPos[7 +1]  - mJointsPos[7 +1];
    errorSignalClean(0) = posErr1;

    if(bDebugMode){
        cout << "Error:   "<< errorSignalClean<<endl;
    }

    // Copy of data for streaming output
    mErrorSignal = errorSignalClean;

    // The position-force switch parameter
    double fsSwitch = 1.0; //(default value)
    mCtrlSwitch = fsSwitch;
    
    // If we have weights, it means that the regression has been performed and 
    // that we can check if we are in the model or not
    if(weights!=NULL){
        if(weights->RowSize()>0){

            double norm = errorSignalClean.Norm();

            // Do the math described in the paper with the position error elements
            tmpErrorSignalClean = ((*weights) * errorSignalClean);
            fsSwitch  = exp(-0.5*(errorSignalClean.GetSubVector(0,7).Dot((*weights).GetMatrix(0,6,0,6)*errorSignalClean.GetSubVector(0,7))));
            mCtrlSwitch = fsSwitch;
            
            // A little tuning, and we have our switch value
            fsSwitch = (1.0 - fsSwitch)*(1.0-fsSwitch);
            
            if(bDebugMode){
                cout <<"Error Norm: "<< fsSwitch<<endl;
            }
        }
    }


    if(bDebugMode){
        cout << "TrError: "<< tmpErrorSignalClean<<endl;
    }

    // What's that ?!?    
    transErrorSignalClean0 = errorSignalClean;

    
    transErrorSignalClean = transErrorSignalClean0;//+= (transErrorSignalClean0-transErrorSignalClean)*0.02/0.2;



// Proportional and integral control limits
#define P_ERRMAX    15
#define I_ERRMAX   800

    // Computing controls
    for(int i=0;i<FINGERS_COUNT;i++){
        // Proximal joints

        // Proporional
        double posErr0              = TRUNC(transErrorSignalClean(1+i*2 + 0),-P_ERRMAX,P_ERRMAX);
        // Derivative
        double dPosErr0             = posErr0 - mPrevSFingerPosErr[i][0];
        // Integral
        double iPosErr0             = TRUNC(mIntSFingerPosErr[i][0] + posErr0,-I_ERRMAX,I_ERRMAX);

        // Storage of previous err for the derivative part
        mPrevSFingerPosErr[i][0]    = posErr0;

        // Storage of previous err for the integral part
        mIntSFingerPosErr[i][0]     = fsSwitch*iPosErr0;
        
        // Distal joints
        double posErr1              = TRUNC(transErrorSignalClean(1+i*2 + 1),-P_ERRMAX,P_ERRMAX);
        double dPosErr1             = posErr1 - mPrevSFingerPosErr[i][1];
        double iPosErr1             = TRUNC(mIntSFingerPosErr[i][1] + posErr1,-I_ERRMAX,I_ERRMAX);
        mPrevSFingerPosErr[i][1]    = posErr1;
        mIntSFingerPosErr[i][1]     = fsSwitch*iPosErr1;

        // Let's gain all these controls
        errorSignal(1+i*2 + 0)       = (mJointsPidKpP[9+i*2 +0] *posErr0);
        errorSignal(1+i*2 + 1)       = (mJointsPidKpP[9+i*2 +1] *posErr1);
        dErrorSignal(1+i*2 + 0)      = (mJointsPidKdP[9+i*2 +0]*dPosErr0);
        dErrorSignal(1+i*2 + 1)      = (mJointsPidKdP[9+i*2 +1]*dPosErr1);
        iErrorSignal(1+i*2 + 0)      = (mJointsPidKdI[9+i*2 +0]*iPosErr0);
        iErrorSignal(1+i*2 + 1)      = (mJointsPidKdI[9+i*2 +1]*iPosErr1);

        // Pressure control
        if(bPidUsePressure){
            // Proportional
            double tipErr                       = transErrorSignalClean(1+FINGERS_COUNT*2+i);
            // Derivative
            double dTipErr                      = tipErr - mPrevSFingerTipErr[i][0];
            // Storage of error for derivative component
            mPrevSFingerTipErr[i][0]            = tipErr;

            // A little asymetric gain for stronger force in the closing direction
            double alpha = 1.0;
            if(tipErr<=0.0) alpha = 0.1;

            // A bit more strength again for the thumb, where the first term is a consant proportional value
            if(i==0){
                errorSignal(1+FINGERS_COUNT*2+i)    = (30.0*mSFingerDesTip[i][0] + mJointsPidKpS[9+i*2 +0] *tipErr)*alpha;                
            }else{
                errorSignal(1+FINGERS_COUNT*2+i)    = (15.0*mSFingerDesTip[i][0] + mJointsPidKpS[9+i*2 +0] *tipErr)*alpha;                            
            }	
            dErrorSignal(1+FINGERS_COUNT*2+i)   = (mJointsPidKdS[9+i*2 +0]*dTipErr)*alpha;                
        }
    }
    // Same for thumb opposition
           posErr1          = TRUNC(transErrorSignalClean(0),-P_ERRMAX,P_ERRMAX);
    double dPosErr1         = posErr1 - mPrevThumbAbdPosErr;
    double iPosErr1         = TRUNC(mIntThumbAbdPosErr + posErr1,-I_ERRMAX,I_ERRMAX);
    mPrevThumbAbdPosErr     = posErr1;
    mIntThumbAbdPosErr      = fsSwitch*iPosErr1;

    errorSignal(0)          = (mJointsPidKpP[7 +1] *posErr1);
    dErrorSignal(0)         = (mJointsPidKdP[7 +1]*dPosErr1);
    iErrorSignal(0)         = (mJointsPidKdI[7 +1]*iPosErr1);

    // Apply the global gain
    errorSignal  *= mJointsPidGain*mJointsPidGainGlobal;
    dErrorSignal *= mJointsPidGain*mJointsPidGainGlobal*mJointsPidGainGlobal;        
    iErrorSignal *= mJointsPidGain*(bPidUseI?1.0:0.0);  



    if(bDebugMode){
        cout << "Cmd:     "<< errorSignal <<endl;
        cout << "DCmd:    "<< dErrorSignal <<endl;
        cout << "ICmd:    "<< iErrorSignal <<endl;
    }

    // Create a matrix that map the control signals to the joint motors
    MathLib::Matrix map(7,10);
    for(int i=0;i<7;i++) map(i,i) = 1.0;
    for(int i=0;i<3;i++) map(1+i*2,7+i) = map(1+i*2+1,7+i) = 1.0;

    // Create a nice control vecotr and truncate the integrative and complete terms
    MathLib::Vector indOutput;
    iErrorSignal.Trunc(-1333.0,1333.0);
    indOutput = errorSignal+dErrorSignal+iErrorSignal;
    indOutput.Trunc(-1333.0,1333.0);

    // Apply the force-position swtich factor
    for(int i=0;i<7;i++) indOutput(i) *= fsSwitch;
    for(int i=0;i<3;i++) indOutput(7+i) *= (1.0-fsSwitch);

    // Compoute the final controls to the joints
    MathLib::Vector output;
    output = map*indOutput; 
    for(int i=0;i<7;i++)
        mJointsTargetCmd[8 +i] = output(i)*mJointsPidFactor[8+i];

    
    // Last joint (actually the first, but this is just to keep the finger's spear roughly constant)
    double posErr0              = mJointsOutputPos[7 +0]  - mJointsPos[7 +0];
    double dPosErr0             = posErr0 - mPrevFingerSpreadPosErr;
    mPrevFingerSpreadPosErr     = posErr0;
    mJointsTargetCmd[7 +0]      = (mJointsPidKpP[7 +0]*posErr0 + mJointsPidKdP[7 +0] *dPosErr0);
    mJointsTargetCmd[7 +0]     *=   mJointsPidFactor[7 +0] * mJointsPidGain;        
    
    
    // Send the open-loop controls to the motors
    if(mOpenLoopController){
        for(int i=mJointsFingerOffset;i<mJointsSize;i++){
            if(mJointControlMode[i]!=2){
                mControlModeController->setOpenLoopMode(i);
                mJointControlMode[i] = 2;
            }           
            double val = TRUNC(floor(mJointsTargetCmd[i]),-1300,1300);
            mOpenLoopController->setOutput(i, val);
        }
    }            

    bDebugMode = bdm;
}


void HandSkinControllerThread::SetDriver(PolyDriver* driver){
    mDriver = driver;
}

void HandSkinControllerThread::SetCtrlMode(HandSkinControllerThread::CtrlMode mode){
    mMutex.wait();
    // Set the control mode
    mCtrlMode = mode;
    // Apply some action in case
    if(mode==CM_IDLE){
        Stop();
    }
    if((mode==CM_REPLAY)||(mode==CM_REPLAYSIM)){
        mReplayStartTime = Time::now();
        bReplayDone      = false;
    }    
    mMutex.post();
}

void HandSkinControllerThread::Stop(){
    // Set current position and tell the PID controller to stay there
    mEncoders->getEncoders(mJointsPos.data());
    for(int i=mJointsFingerOffset;i<mJointsSize;i++){
        mPIDController->setReference (i, mJointsPos[i]);
    }
}

void HandSkinControllerThread::ReadFromPort(){
    // Read skin data from input port
    Vector *inputVec;
    inputVec = mFingerTipPort.read(false);
    if(inputVec!=NULL){
        if(!((inputVec->size()==SKIN_FULL_SIZE)||(inputVec->size()==SKIN_SIZE))){
            fprintf(stderr,"Bad vector size as input to fingertip:i port\n");
        }else{
            mFingerTipRaw = *inputVec;

            for(int i=0;i<inputVec->size();i++){
                mFingerTipRaw[i] = 255 - mFingerTipRaw[i];
            }                               

            if(!bFilterSkin){
                // No filtering, just copy data
                for(int i=0;i<SKIN_FULL_SIZE;i++){
                    double val = mFingerTipRaw[i];
                    mFingerTip[i] = (val>0.0?val:0.0);
                }                               
            }else{
                // Filtering first then copy data
                for(int i=0;i<SKIN_FULL_SIZE;i++){
                    mFingerTip[i] += (-mFingerTip[i] + mFingerTipRaw[i]) * 0.02/mSkinFilterTau;
                    double val = mFingerTip[i];
                    mFingerTip[i] = (val>0.0?val:0.0);                    
                }            
            }

            bGotSkin = true;
        }
    }
} 

// Read sensor data and prepare SFinger data structure
void HandSkinControllerThread::PrepareSFingers(){

    bool bdm = bDebugMode;
    bDebugMode = false;

    if(bDebugMode){
        cout << "Input *************"<<endl;
    }

    // Position and default command
    for(int i=0;i<FINGERS_COUNT;i++){
        mSFingerPos[i][0] = mJointsPos[9+i*2 +0];
        mSFingerPos[i][1] = mJointsPos[9+i*2 +1];
        
        mSFingerCmd[i][0] = mJointsPos[9+i*2 +0];
        mSFingerCmd[i][1] = mJointsPos[9+i*2 +1];
    }

    static double badSkinVal[12];
    double tmpSkinVal[12];
    if(bBadSkin){
        // Let simulate some bad skin values from time to time
        //double period = 0.2;
        //double off = mTime/period - floor(mTime/period);
        //if(off < 0.05){
            for(int j=0;j<12;j++){
                badSkinVal[j] = 9.0+4.0*(rand()%1000)/1000.0;
            }
        //}
        
        for(int i=0;i<FINGERS_COUNT;i++){
            if(i==mBadFingerId){
                int offset = (i==0?4:i-1)*12;
                for(int j=0;j<12;j++){
                    tmpSkinVal[j] = mFingerTip[offset+j]; 
                    mFingerTip[offset+j] = badSkinVal[j];
                }
            }
        }
           
    }


    // Compute the average fingerTip pressure    
    for(int i=0;i<FINGERS_COUNT;i++){
        int offset = (i==0?4:i-1)*12;
        double val = 0.0;
        for(int j=0;j<12;j++){
            val += mFingerTip[offset+j];
        }
        val /= double(12);     
        mSFingerTip[i][0] = val;        
    }

    if(bDebugMode){
        cout << "Input Pressure:  "<< mSFingerTip[0][0]<<" "<<mSFingerTip[1][0]<<" "<<mSFingerTip[2][0]<<endl;
    }


    // FingerTip normal direction
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
    MathLib::Matrix fingerTipPDir;
    fingerTipPDir.Set(vals,12,3);


    // Compute the normal direction
    for(int i=0;i<FINGERS_COUNT;i++){
        mSFingerTipDir[i].Zero();
        int offset = (i==0?4:i-1)*12;
        double val = 0.0;
        for(int j=0;j<12;j++){
            val               += mFingerTip[offset+j];
            mSFingerTipDir[i] += fingerTipPDir.GetRow(j) *(mFingerTip[offset+j]);
        }        
        if(val>1.0){
            mSFingerTipDir[i].Normalize();
        }else{
            mSFingerTipDir[i].Zero();
        }
    }


    if(bBadSkin){
        // Revert original skin values if necessary for data saving
        for(int i=0;i<FINGERS_COUNT;i++){
            if(i==mBadFingerId){
                int offset = (i==0?4:i-1)*12;
                for(int j=0;j<12;j++){
                    mFingerTip[offset+j] = tmpSkinVal[j];
                }
            }
        }
           
    }


    if(bDebugMode){
        cout << "Input Direction: "<< mSFingerTipDir[0]<<" "<<mSFingerTipDir[1]<<" "<<mSFingerTipDir[2]<<endl;
    }
    
    bDebugMode = bdm;
    return;

}



void HandSkinControllerThread::ResetSFingerData(){
    for(int i=0;i<FINGERS_COUNT;i++){
        // Set the correct size of all temporary cariables
        mSFingerPos[i].Resize(2);
        mSFingerCmd[i].Resize(2);
        mSFingerTip[i].Resize(1);
        mSFingerTipDir[i].Resize(3);


        mSFingerDesPos[i].Resize(2);
        mSFingerDesTip[i].Resize(1);
        
        mPrevSFingerTipErr[i].Resize(1);
        mPrevSFingerPosErr[i].Resize(2);
        mIntSFingerPosErr[i].Resize(2);

        // Default values                
        mSFingerPos[i]      = 10;
        mSFingerTip[i]      = 0;
        mSFingerCmd[i]      = 10;
        mSFingerTipDir[i]   = 0;

        mSFingerDesPos[i]   = 10;
        mSFingerDesTip[i]   = 0;

        mPrevSFingerTipErr[i] = 0;
        mPrevSFingerPosErr[i] = 0;
        mIntSFingerPosErr[i]  = 0;

    }
    mSFingerDesTip[0][0] = 45;
    mSFingerDesTip[1][0] = 30;
    mSFingerDesTip[2][0] = 17;
    
    mPrevThumbAbdPosErr     = 0;
    mPrevFingerSpreadPosErr = 0;
    mIntThumbAbdPosErr      = 0;
}


// Min skin response threshold
#define FTIPTHRESHOLD       20.0
// Max skin response threshold
#define FTIPTHRESHOLDMAX    25.0

void    HandSkinControllerThread::PredictPoseFromAngle(){
    // Apply the model and rock the field
    bool bdm = bDebugMode;
    bDebugMode = false;

    if(bDebugMode) cout << "Predicting pose from angle *********"<<endl;


    // Count the number of valid contact

    // Reliability measure for each fingertip
    double  validFTip[FINGERS_COUNT];
    int     validFTipCount = 0;

    if(bNoMissingCheck){
        // If we don't care, let's move on
        for(int i=0;i<FINGERS_COUNT;i++){
            validFTip[i] = 1.0;        
        }
        validFTipCount = FINGERS_COUNT;
    }else{
        for(int i=0;i<FINGERS_COUNT;i++){
            // Higher than min threshold?
            if(mSFingerTip[i][0] >= FTIPTHRESHOLD){
                validFTipCount++;
                // Higher than max threshold?
                if(mSFingerTip[i][0] >= FTIPTHRESHOLDMAX){
                    validFTip[i] = 1.0;
                }else{
                    // Save validity between 0 and 1
                    validFTip[i] = (mSFingerTip[i][0]-FTIPTHRESHOLD)/(FTIPTHRESHOLDMAX-FTIPTHRESHOLD);
                }
            }else{
                validFTip[i] = 0.0;        
            }
        }
    }



    // Additional input weight matrix using the realiability measure
    MathLib::Matrix     inWeights(validFTipCount*3,validFTipCount*3);    
    inWeights.Zero();
    int cnt = 0;
    for(int i=0;i<FINGERS_COUNT;i++){
        if(validFTip[i]>0){
            inWeights(cnt*3+0,i*3+0) = validFTip[i];
            inWeights(cnt*3+1,i*3+1) = validFTip[i];
            inWeights(cnt*3+2,i*3+2) = validFTip[i];
            cnt++;
        }
    }
    
    // Some variables
    MathLib::Vector     inComp(validFTipCount*3);
    MathLib::Vector     outComp(FINGERS_COUNT*2+1+FINGERS_COUNT*1);        
    MathLib::Vector     inV(validFTipCount*3);
    MathLib::Vector     outV(FINGERS_COUNT*2+1+FINGERS_COUNT*1); outV.Zero();
    MathLib::Matrix     sigma(FINGERS_COUNT*2+1+FINGERS_COUNT*1,FINGERS_COUNT*2+1+FINGERS_COUNT*1);

    // Output components indices of the gmm
    for(int i=0;i<outComp.Size();i++)
        outComp(i) = i;
    

    // Input components indices of the gmm
    cnt = 0;
    for(int i=0;i<FINGERS_COUNT;i++){
        if(validFTip[i]>0){
            inComp(cnt++) = outComp.Size()+i*3+0;
            inComp(cnt++) = outComp.Size()+i*3+1;
            inComp(cnt++) = outComp.Size()+i*3+2;
        }
    }

    if(bDebugMode){ cout <<"Vld:  "; for(int i=0;i<FINGERS_COUNT;i++) cout << validFTip[i]<<" "; cout<<endl;}
    if(bDebugMode) cout <<"InC:  " << inComp<<endl;
    
    // Filling the input vector with sensor data
    cnt = 0;
    for(int i=0;i<FINGERS_COUNT;i++){
        if(validFTip[i]>0){
            inV(cnt++) = mSFingerTipDir[i][0];
            inV(cnt++) = mSFingerTipDir[i][1];
            inV(cnt++) = mSFingerTipDir[i][2];
        }
    }

    if(bDebugMode) cout <<"InV:  "<< inV<<endl;


    MathLib::Vector grad,gradNoExp;
    MathLib::Vector inV2,inV0;
    inV2 = inV;
    inV0 = inV;
    

    if(validFTipCount>0){
        // We have at least one valid input
        if(bGradientAscent){
            // We enjoy gradient ascent

            // Get the probability that input query point is in the model            
            double val = mGMModel.GetPx(inV2,inComp,&inWeights);
            
            // Iterate until we're happy
            int iter = 0;
            int iterNoExp = 0;            
            while(1){
                // Get the gradient of current point
                val = mGMModel.GetGradX(inV2,inComp,grad,&inWeights,&gradNoExp);

                // Get the membership of the current input query point, 
                // if higher than threshold we're done 
                if(val >= exp(-0.5*2.5*2.5))
                    break;

                // Norm of the gradient
                double norm = grad.Norm();
                if(val<1e-12){
                    // Too small? use another safer version in gradNoExp
                    grad = gradNoExp;
                    norm = gradNoExp.Norm();
                    norm = 1.0/norm; 

                    iterNoExp++;

                    if(!isnormal(norm)){
                        // Still a not - number... big failure... but we 
                        // exit still...
                        iter = -1;                    
                        break;
                    }
                }else{
                    // Big enough? Normalize
                    norm = 1.0/norm;                
                }
                // Normalize
                grad *= (-norm);
                // Scale the gradient to a little step size and add it 
                // to the query point
                grad.ScaleAddTo(0.01,inV2);

                // Normalization of the query point (i.e. the contact normals
                // should be better unit norm
                int cnt2 = 0;
                for(int i=0;i<FINGERS_COUNT;i++){
                    if(validFTip[i]>0){
                        MathLib::Vector3 vn;
                        vn(0) = inV2(cnt2+0);
                        vn(1) = inV2(cnt2+1);
                        vn(2) = inV2(cnt2+2);
                        vn.Normalize();
                        inV2(cnt2+0) = vn(0);
                        inV2(cnt2+1) = vn(1);
                        inV2(cnt2+2) = vn(2);
                        cnt2 += 3;
                    }
                }

                iter++;
                if(iter>300){
                    // Too much iteration.... again bg failure, exit gracefully
                    break;
                }
            }
            if(bDebugMode) cout << "  Gradient Descent: # iterations : "<<iter<< "("<<iterNoExp<<")"<<endl;


            if((iter<0)||(iter>300)){
                mSFingerDesCoupling.Identity();
                if(bDebugMode){
                    cout << "  Failed: Membership Value (threshold: "<<exp(-0.5*2.5*2.5)<<") -> "<<val <<endl;
                    cout << endl<<endl;
                }
                bDebugMode= bdm;
                return;
            }
        }
    }
    
    
    if(bDebugMode) cout <<"Input: "<< inV<<endl;
    
    // Finally do the regression :)
    inV = inV2;
    double res   = mGMModel.doRegression( inV , inComp, outComp, outV, sigma,&inWeights);

    if(bDebugMode) cout << "Confidence: "<<res<<endl;    
    if(bDebugMode) cout << "Output: "<<outV<<endl;

    // Desired position
    for(int i=0;i<FINGERS_COUNT;i++){
        mSFingerDesPos[i][0] = outV(1+i*2+0);// / mPosFactor;
        mSFingerDesPos[i][1] = outV(1+i*2+1);// / mPosFactor;
    }
    // Desired pressure
    for(int i=0;i<FINGERS_COUNT;i++){
        mSFingerDesTip[i][0] = outV(1+FINGERS_COUNT*2+i)+2.0;
        
        // Tweaking the output sigma matrice (identiting the pressure part,
        // and keeping the position part only (for the force-position control 
        // switch))
        sigma.SetRow(0.0,1+FINGERS_COUNT*2+i);
        sigma.SetColumn(0.0,1+FINGERS_COUNT*2+i);
        sigma(1+FINGERS_COUNT*2+i,1+FINGERS_COUNT*2+i) = 1.0;
    }

    // Inverting the sigma matrix
    MathLib::Matrix nsigma(sigma);
    nsigma.Identity();
    nsigma *= 10.0; // A bit more power in the diagonal
    nsigma += sigma;
        
    // Inverse and store it for later use
    nsigma.InverseSymmetric(mSFingerDesCoupling);    
    

    // More coping fo data here and there
    mJointsTargetPredPos = mJointsRest;
    for(int i=mJointsFingerOffset+1;i<mJointsSize-1;i++){
        mJointsTargetPredPos(i) = outV(i-mJointsFingerOffset-1);
    }

    bDebugMode= bdm;    
}



// Send the data to the output port
void    HandSkinControllerThread::SendOutputData(){
    // Testing mode or not
    if(!bTestingOutput){
        mOutputData.resize(mJointsSize-mJointsFingerOffset + FINGERS_COUNT + FINGERS_COUNT*2 + FINGERS_COUNT*3);
        mTrainingData.Resize(40000,mOutputData.size(),false);
    }else{
        mOutputData.resize(mJointsSize-mJointsFingerOffset + FINGERS_COUNT + FINGERS_COUNT*2 + FINGERS_COUNT*3 + 12*5 + FINGERS_COUNT*2+1+FINGERS_COUNT*1 + 1);
        mTrainingData.Resize(40000,mOutputData.size(),false);
    }
    
    mOutputData = 0;
    int off = 0;
    // First dump the joint angles
    for(int i=mJointsFingerOffset;i<mJointsSize;i++)
        mOutputData[off + i-mJointsFingerOffset] = mJointsPos[i];

    // Then the average pressure values
    off = mJointsSize-mJointsFingerOffset;
    for(int i=0;i<FINGERS_COUNT;i++)
        mOutputData[off +                   i  ] = mSFingerTip[i][0];
    off += FINGERS_COUNT;

    // Then some old stuff such as an angle (still here for historical reasons)
    for(int i=0;i<FINGERS_COUNT;i++){        
        mOutputData[off +   2*i+0] = atan2(mSFingerTipDir[i][1],mSFingerTipDir[i][2])*180.0/3.14;
        mOutputData[off +   2*i+1] = atan2(mSFingerTipDir[i][0],mSFingerTipDir[i][2])*180.0/3.14;
    }
    off += 2*FINGERS_COUNT;

    // Then the normal direction
    for(int i=0;i<FINGERS_COUNT;i++){        
        mOutputData[off + 3*i+0] = mSFingerTipDir[i][0];
        mOutputData[off + 3*i+1] = mSFingerTipDir[i][1];
        mOutputData[off + 3*i+2] = mSFingerTipDir[i][2];        
    }
    off += FINGERS_COUNT*3;


    // If we are in testing mode, some more stuff
    if(bTestingOutput){
        // Raw, unordered (sorry) response of individual sensor
        for(int i=0;i<12*5;i++){        
            mOutputData[off+i] = mFingerTip[i];
        }
        off += 12*5;
                
        // The error signals computed somewhere else by the position-force controller
        for(int i=0;i<FINGERS_COUNT*2+1+FINGERS_COUNT*1;i++){
            mOutputData[off+i] = mErrorSignal(i);
        }
        off += FINGERS_COUNT*2+1+FINGERS_COUNT*1;
        
        // The force-position control switch (which also say if we are right in the model or not)
        mOutputData[off] = mCtrlSwitch;
    }

    
    if(bSendOutputData){
        // If we are streaming data, add data to the storage matrix
        if(mTrainingDataPos>=0){
            if(mTrainingDataPos<40000){
                for(int k=0;k<mOutputData.size();k++){
                    mTrainingData(mTrainingDataPos,k) = mOutputData[k];
                }
                mTrainingDataPos ++;
            }
        }
    
        Vector& vec = mOutputDataPort.prepare();
        vec = mOutputData;
        mOutputDataPort.write();
    }

}


void    HandSkinControllerThread::SwitchSendOutputData(){
    // Switch between sening data to the output port or not
    bSendOutputData = !bSendOutputData;
    if(bSendOutputData){
        fprintf(stderr,"Sending data\n");
        mTrainingDataPos = 0;
    }else{
        fprintf(stderr,"Stop sending data\n");
        if(mTrainingDataPos>=0){
            if(bAutoSaveTrainingData){
                char txt[256];
                snprintf(txt,256,"%s/%s.txt",mBasePath,mTrainingDataName);            
                if(mTrainingData.Save(txt,6,mTrainingDataPos)){
                    cout << "File: "<<txt<<" saved: "<<mTrainingDataPos<<" rows"<<endl;
                }else{
                    cout << "Error while saving file: "<<txt<<endl;
                }
            }
        }
        mTrainingDataPos = -1;
    }
}


void HandSkinControllerThread::SetBasePath(const char *path){
    strncpy(mBasePath,path,256);
}







void HandSkinControllerThread::ResetSkin(){
    if(mDriver==NULL) return;

    fprintf(stderr,"ResetSkin: Resetting skin calibration\n");
    char part[256];
    strcpy(part,mPartName);

    IAnalogSensor   *tactileSensor;	
    PolyDriver      *tactileSensorDevice;

    if((strcmp(part,"left")!=0)&&(strcmp(part,"right")!=0)){
        fprintf(stderr, "ResetSkin: Error: Bad parts specifed (%s)\n",part);
        fprintf(stderr, "ResetSkin: Error:   Please choose one: (right or left)\n");
    }
    

    char txt[256];
        
    Property options;
    options.put("robot",  "icub");
    sprintf(txt,"%s_hand",part);            options.put("part",   txt);         //skin part that you want to control
    sprintf(txt,"/skinReset/%s",part);      options.put("local",  txt);

    char portName2[256];
    snprintf(portName2,256,"/icub/skin/%s_arm",mPartName);
    if(yarp::os::Network::exists(portName2)){
        sprintf(txt,"/icub/skin/%s_arm",part);  options.put("remote", txt);
    }else{
        sprintf(txt,"/icub/skin/%s_hand",part);  options.put("remote", txt);
    }
    
    options.put("device", "analogsensorclient");
	 
    // create a new device driver
    tactileSensorDevice = new PolyDriver(options);
    if(tactileSensorDevice!=NULL){
        if (!tactileSensorDevice->isValid()){
	        fprintf(stderr,"ResetSkin: Error: Device not available.  Here are the known devices:\n");
		    fprintf(stderr,"%s", Drivers::factory().toString().c_str());
            tactileSensorDevice->close();
            delete tactileSensorDevice;
            return;
        }
        // open the sensor interface	
        bool ok = tactileSensorDevice->view(tactileSensor);
        if (!ok) {
	        fprintf(stderr,"ResetSkin: Error: Problems acquiring interfaces\n");
        }
        usleep(200*1000);
        int dim = tactileSensor->getChannels();
        
        if(dim==0){
	        fprintf(stderr,"ResetSkin: Error: while reading the number of channels of the tactile sensor device\n");
        }else{
            fprintf(stderr,"ResetSkin: Got %d sensors on part %s\n",dim,part);
        }
        // Reset the sensor family
        tactileSensor->calibrateSensor();	
        tactileSensorDevice->close();
        delete tactileSensorDevice;
    }
    mFingerTipCalibCount = 0;
}

