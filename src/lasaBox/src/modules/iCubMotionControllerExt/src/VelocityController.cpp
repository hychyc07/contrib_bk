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

#include "VelocityController.h"
#include <cstring>
#include <iostream>
using namespace std;

#define POS_OFFSET_TIME (0.08    )

VelocityController::PositionElement::PositionElement(int size){
    Set(size);
    bValid = false;
}
VelocityController::PositionElement::PositionElement(const VelocityController::PositionElement & p){
    Set(p.mPos,p.mTime,p.bValid);
}
VelocityController::PositionElement::PositionElement(const Vector& pos, double time, bool valid){
    Set(pos,time,valid);
}

void    VelocityController::PositionElement::Set(int size){
    if(size>0)
        mPos.resize(size,0.0);
    else
        mPos.clear();
    mTime = 0.0;
    bValid = false;
}
void    VelocityController::PositionElement::Set(const Vector& pos, double time){
    mPos  = pos;
    mTime = time;
}
void    VelocityController::PositionElement::Set(const Vector& pos, double time, bool valid){
    Set(pos,time);
    bValid = valid;
}
bool    VelocityController::PositionElement::IsValid(){
    return bValid;
}


VelocityController::VelocityController(){
    bIsReady        = false;
}

VelocityController::~VelocityController(){
    Free();
}


bool VelocityController::Init(PolyDriver *driver, const char* name, const char* basename){
    fprintf(stderr,"********************************************\n");
    if(driver==NULL){
        fprintf(stderr,"Warning: Null driver\n");
    }
    mDriver = driver;
    
    if(basename!=NULL){
        strcpy(mBaseName,basename);
    }else{
        mBaseName[0] = 0;
    }
    if(name!=NULL){
        strcpy(mName,name);
    }else{
        strcpy(mName,"MCE");
    }
    

  
    bool bOk = true;
    if(mDriver){
        bOk &= mDriver->view(mEncoders);  
        bOk &= mDriver->view(mVelocityController);
        bOk &= mDriver->view(mPositionController);
        bOk &= mDriver->view(mLimitsController);
        bOk &= mDriver->view(mPIDController);
        bOk &= mDriver->view(mControlModeController);
    }else{
        mEncoders               = NULL;
        mLimitsController       = NULL;
        mVelocityController     = NULL;
        mPositionController     = NULL;
        mPIDController          = NULL;
        mControlModeController  = NULL;
    }
    if(!bOk){
        fprintf(stderr,"Error: Problem getting drivers interfaces\n");
        return false;    
    }
    
    fprintf(stderr,"Starting Local Controller: <%s/%s>\n",mBaseName,mName);
    if(mVelocityController){
        mVelocityController->getAxes(&mJointsSize);
    }else{
        fprintf(stderr,"  Using a fake controller...\n");
              if (strcmp(mName,"fake_right_arm")==0){
            mJointsSize = 16;
        }else if (strcmp(mName,"fake_left_arm")==0){
            mJointsSize = 16;
        }else if (strcmp(mName,"fake_head")==0){
            mJointsSize = 6;
        }else if (strcmp(mName,"fake_torso")==0){
            mJointsSize = 3;
        }else if (strcmp(mName,"fake_right_leg")==0){
            mJointsSize = 6;
        }else if (strcmp(mName,"fake_left_leg")==0){
            mJointsSize = 6;
        }else{
            mJointsSize = 1;
        }
    }
    fprintf(stderr,"  # Joints: %d\n",mJointsSize);

    if(mVelocityController){
        Vector accs; accs.resize(mJointsSize);
        accs = 100000;
        mVelocityController->setRefAccelerations(accs.data());
    }
    
    if(mPositionController){
        Vector vels; vels.resize(mJointsSize);
        vels = 50;
        mPositionController->setRefSpeeds(vels.data());
    }
    
    mJointsPos.resize(mJointsSize);
    mJointsVel.resize(mJointsSize);

    mJointsTargetPos.resize(mJointsSize);
    mJointsTargetVel.resize(mJointsSize);

    mJointsInternalPos.resize(mJointsSize);
    mJointsInternalVel.resize(mJointsSize);
    mJointsStartPos.resize(mJointsSize);

    mJointsOutputVel.resize(mJointsSize);
    mJointsOutputPos.resize(mJointsSize);
    mJointsPrevOutputVel.resize(mJointsSize);

    mJointsMask.resize(mJointsSize);
    mJointsError.resize(mJointsSize);

    mJointsKp.resize(mJointsSize);
    mJointsKd.resize(mJointsSize);

    mJointsPosLimits[0].resize(mJointsSize);
    mJointsPosLimits[1].resize(mJointsSize);
    mJointsVelLimits[0].resize(mJointsSize);
    mJointsVelLimits[1].resize(mJointsSize);
    mJointsRange.resize(mJointsSize);    
    for(int i=0;i<mJointsSize;i++){
        if(mLimitsController){
            mLimitsController->getLimits(i,mJointsPosLimits[0].data()+i,mJointsPosLimits[1].data()+i);
        }
        mJointsRange[i] = mJointsPosLimits[1][i] - mJointsPosLimits[0][i];
    }
    
    mJointsRest.resize(mJointsSize);
    cout << mJointsPosLimits[1].toString()<<endl;
    cout << mJointsPosLimits[0].toString()<<endl;
    mJointsKp = 1.5;
    mJointsKd = 0;    
    mJointsMask = 1.0;
    
    mJointsTargetPos = 0;    
    mJointsTargetVel = 0;    
    mJointsError     = 0;
    
    mMode               = VC_IDLE;

    mTime               = 0.0;
    mPrevTime           =-1.0;
    mLastPosCtrlTime    = 0.0;

    mCommandTimeout         = 0.2;
    mMinimumLoopTime        = 0.005;
    mCummulativeDt          = 0.0;    
    mLastPosCommandTime     = mCommandTimeout + 0.1;
    mLastVelCommandTime     = mCommandTimeout + 0.1;
    mLastIdleCommandTime    = mTime;
    
    mJointsVelLimits[0] = -75.0;
    mJointsVelLimits[1] = +75.0;
    
    bIdleModeSet            = true;
    bPosTargetSet           = false;
    bVelTargetSet           = false;
    bPosTimeoutPause        = false;
    bVelTimeoutPause        = false;
    bIdleTimeoutPause       = false;

    
    bFirst      = true;
    bIsReady    = true;
    bReset      = true;

    mIdleStep   = 0;
    
    while(!mPosQueue.empty()) mPosQueue.pop();
    mFirstPos.Set(mJointsSize);
    mLastPos.Set(mJointsSize);

    mDebugPort.open("/MCE/debug");

    fprintf(stderr,"********************************************\n");
    return true;
}


void VelocityController::Free(){
    Stop();
    mDebugPort.close();
    bIsReady = false;
}


void VelocityController::Update(){
    if(!bIsReady) return;
    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        return;
    }else{
        mPrevTime = mTime;    
    }
    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;
    
    Update(dt);    
    
}

#define POSCTRL_TIMETOGO 0.5

void VelocityController::Update(double dt){
    if(!bIsReady) return;

    mTime       = Time::now();

    if(dt < mMinimumLoopTime){
        mCummulativeDt += dt;
        cout << "Too fast: "<< dt << "<" <<mMinimumLoopTime<<endl;
        if(mCummulativeDt < mMinimumLoopTime){
            mMutex.post();
            return;    
        }else{
            dt = mCummulativeDt;
        }        
    }
    mCummulativeDt = 0.0;
    double invDt = 1.0/dt;

    mMutex.wait();

    if(mEncoders){
        mEncoders->getEncoders              (mJointsPos.data());
        mEncoders->getEncoderSpeeds         (mJointsVel.data());
    }
    
    if(bFirst){
        mJointsRest             = mJointsPos;
        bFirst                  = false;
    }
    if(bReset){
        mJointsInternalPos      = mJointsPos;
        mJointsInternalVel      = 0.0;

        mJointsError            = 0.0;
        mJointsPrevOutputVel    = 0.0;

        mFirstPos.Set(mJointsInternalPos,mTime +       POS_OFFSET_TIME, false);
        mLastPos.Set( mJointsInternalPos,mTime + 2.0 * POS_OFFSET_TIME, false);
        mPosTimeToGo    = POS_OFFSET_TIME;
        mJointsStartPos = mJointsInternalPos;

        bReset                  = false;
    }


    // Get commands
    if(bPosTargetSet){
        mLastPosCommandTime = 0.0;
        bPosTargetSet       = false;
    }else{
        if(mLastPosCommandTime<=mCommandTimeout)
            mLastPosCommandTime += dt;
    }
    if(bVelTargetSet){
        mLastVelCommandTime = 0.0;
        bVelTargetSet       = false;
    }else{
        if(mLastVelCommandTime<=mCommandTimeout)
            mLastVelCommandTime += dt;
    }
    if(bIdleModeSet){
        mLastIdleCommandTime    = 0.0;
        bIdleModeSet            = false;
    }else{
        if(mLastIdleCommandTime<=mCommandTimeout)
            mLastIdleCommandTime   += dt;
    }
    

    // Check for last command time
    if(mLastPosCommandTime>mCommandTimeout){
        if(!bPosTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: pausing position control..."<<endl;
            bPosTimeoutPause = true;            
        }
        mJointsTargetPos = mJointsPos;
        //mJointsPosCtrlTimeToGo = POSCTRL_TIMETOGO;
    }else{
        if(bPosTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Data received: resuming position control..."<<endl;
            bPosTimeoutPause = false;
        }                    
    }    
    // Check for last command time
    if(mLastVelCommandTime>mCommandTimeout){
        if(!bVelTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: pausing velocity control..."<<endl;
            bVelTimeoutPause = true;            
        }
        mJointsTargetVel = 0.0;
    }else{
        if(bVelTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Data received: resuming velocity control..."<<endl;
            bVelTimeoutPause = false;
        }                    
    }    
    // Check for last idle command time    
    if(mLastIdleCommandTime>mCommandTimeout){
        if(!bIdleTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Timeout: entrering full idle mode..."<<endl;
            bIdleTimeoutPause = true;
        }
    }else{
        if(bIdleTimeoutPause){
            cout << mBaseName<<"/"<<mName<<": Idle mode requested: entrering idle mode..."<<endl;
            bIdleTimeoutPause = false;
        }
    }




    // Position checks
    for(int i=0;i<mJointsSize;i++){
        if(mJointsTargetPos[i] < mJointsPosLimits[0][i]){
            mJointsTargetPos[i] = mJointsPosLimits[0][i];
        }else if(mJointsTargetPos[i] > mJointsPosLimits[1][i]){
            mJointsTargetPos[i] = mJointsPosLimits[1][i];
        }
    }


    // Consommation
    if(mFirstPos.IsValid()){
        while(mFirstPos.mTime <= mTime){
            //cout << mFirstPos.mTime  - mTime<<endl;
            if(!mPosQueue.empty()){
                mPosTimeToGo    = mPosQueue.front().mTime - mFirstPos.mTime;
                mJointsStartPos = mFirstPos.mPos;
                mFirstPos       = mPosQueue.front();
                mPosQueue.pop();
            }else{
                mJointsStartPos = mFirstPos.mPos;
                mFirstPos = mLastPos;
                mLastPos.mTime += POS_OFFSET_TIME ;
                mLastPos.bValid = false;
                mPosTimeToGo    = mLastPos.mTime - mFirstPos.mTime;
            }
        }
    }else{
        if(mFirstPos.mTime <= mTime){
            mJointsStartPos = mFirstPos.mPos;
            mFirstPos = mLastPos;
            mLastPos.mTime += POS_OFFSET_TIME ;
            mLastPos.bValid = false;
            mPosTimeToGo    = POS_OFFSET_TIME;
        }
    }
    //cout << 1.0-((mFirstPos.mTime - mTime)/POS_OFFSET_TIME) <<" ";
    //PQueue();
    Vector dtpos;
    dtpos.resize(mJointsSize);
    dtpos = mJointsInternalPos;

    for(int i=0;i<mJointsSize;i++){
        double currError = 0.0;
        double kdPart    = 0.0;
        double kpPart    = 0.0;
        
        double mVelTau   = 0.2;
        double dtOnTau   = dt/mVelTau;
        
        switch(mMode){
        case VC_IDLE:{
            mJointsTargetPos[i] = mJointsPos[i];
            mJointsTargetVel[i] = 0.0;

            mJointsOutputPos[i] = mJointsPos[i];
            mJointsOutputVel[i] = 0.0;
            mJointsPosCtrlTimeToGo = POSCTRL_TIMETOGO;
            break;}

        case VC_REST:{
            mJointsTargetPos[i] = mJointsRest[i];
            mJointsTargetVel[i] = 0.0;

            mJointsOutputPos[i] = mJointsRest[i];
            mJointsOutputVel[i] = 0.0;
        break;}
        case  VC_VELOCITY:{
            mJointsOutputVel[i] = 0.0;
            break;
            /*
            // Velocity control
            double outputVel = mJointsTargetVel[i];

            // Position control
            if(mJointsTargetPos[i]<mJointsPosLimits[0][i]){
                mJointsTargetPos[i] = mJointsPosLimits[0][i];
            }else if(mJointsTargetPos[i]>mJointsPosLimits[1][i]){
                mJointsTargetPos[i] = mJointsPosLimits[1][i];
            }
            
            currError    = mJointsTargetPos[i]-mJointsPos[i];
            if(fabs(currError)<1.5) currError = 0.0;
            kdPart = mJointsKd[i]*(currError - mJointsError[i])*invDt;
                
            mJointsError[i]     = currError;
            kpPart = mJointsKp[i]*(mJointsError[i]);
            //cout << mJointsKd[i]<<" "<<kdPart<<" ";
            outputVel += kpPart + kdPart;

            mJointsOutputVel[i] = outputVel;
            //mJointsOutputVel[i] = mJointsPrevOutputVel[i] + (-mJointsPrevOutputVel[i] + outputVel)*dtOnTau;
            
            double minVelLimit = mJointsVelLimits[0][i];
            double maxVelLimit = mJointsVelLimits[1][i];
            
            if(mJointsPos[i]-mJointsPosLimits[0][i]<5.0){
                if(mJointsPos[i]-mJointsPosLimits[0][i]<0.0) minVelLimit = 0.0;
                else minVelLimit *= (mJointsPos[i]-mJointsPosLimits[0][i])/5.0;
            }else if(mJointsPosLimits[1][i]-mJointsPos[i]<5.0){
                if(mJointsPosLimits[1][i]-mJointsPos[i]<0.0) maxVelLimit = 0.0;
                else maxVelLimit *= (mJointsPosLimits[1][i]-mJointsPos[i])/5.0; 
            }

            if(mJointsOutputVel[i]<minVelLimit){
                mJointsOutputVel[i] = minVelLimit;
            }else if(mJointsOutputVel[i]>maxVelLimit){
                mJointsOutputVel[i] = maxVelLimit;
            }
            break;*/
            }

        case VC_POSITION:{
            double ttg = 1.0-((mFirstPos.mTime - mTime)/POS_OFFSET_TIME);
            double tpos = mJointsStartPos[i] + (mFirstPos.mPos[i]-mJointsStartPos[i])*ttg;
            double vel = (tpos - mJointsInternalPos[i])*invDt;

            double acc = (vel - mJointsInternalVel[i])*invDt;
            double minAccLimit = -2000.0;
            double maxAccLimit =  2000.0;

            if(acc < minAccLimit){
                acc = minAccLimit;
            }else if(acc > maxAccLimit){
                acc = maxAccLimit;
            }
            acc -= 10.0*mJointsInternalVel[i];

            vel = mJointsInternalVel[i] + acc*dt;

            double minVelLimit = mJointsVelLimits[0][i];
            double maxVelLimit = mJointsVelLimits[1][i];
            if(vel < minVelLimit){
                vel = minVelLimit;
            }else if(vel > maxVelLimit){
                vel = maxVelLimit;
            }

            mJointsInternalVel[i] = vel;
            mJointsInternalPos[i] = mJointsInternalPos[i] + vel*dt;

            dtpos[i] = tpos;

            if(mJointsInternalPos[i] < mJointsPosLimits[0][i]){
                mJointsInternalPos[i] = mJointsPosLimits[0][i];
            }else if(mJointsInternalPos[i] > mJointsPosLimits[1][i]){
                mJointsInternalPos[i] = mJointsPosLimits[1][i];
            }

            mJointsOutputPos[i] = mJointsInternalPos[i];//mJointsRest[i];
            break;
            /*
            if(mJointsPosCtrlTimeToGo>0.0){
                double alpha = mJointsPosCtrlTimeToGo/double(POSCTRL_TIMETOGO); 
                mJointsOutputPos[i] = alpha*mJointsPos[i] + (1.0-alpha)*mJointsTargetPos[i];
            }else{
                mJointsOutputPos[i] = mJointsTargetPos[i];
            }

            double vel = (mJointsOutputPos[i]-mJointsPos[i])*invDt;
            double minVelLimit = mJointsVelLimits[0][i];
            double maxVelLimit = mJointsVelLimits[1][i];
            if(vel < minVelLimit){
                mJointsOutputPos[i] = mJointsPos[i] +minVelLimit*dt;
            }else if(vel > maxVelLimit){
                mJointsOutputPos[i] = mJointsPos[i] +maxVelLimit*dt;
            }

            //mJointOutputPos[i] = mJointTargetPos[i];
            break;*/
            }
        }
    }
    Vector & vec = mDebugPort.prepare();
    vec.resize(mJointsSize*3);
    for(int i=0;i<mJointsSize;i++){
        vec[i]             = mJointsInternalPos[i];
        vec[i+mJointsSize] = dtpos[i];
        vec[i+2*mJointsSize] = mJointsPos[i];
    }
    mDebugPort.write();

    mJointsPrevOutputVel = mJointsOutputVel;


    // Position checks
    for(int i=0;i<mJointsSize;i++){
        if(mJointsOutputPos[i] < mJointsPosLimits[0][i]){
            mJointsOutputPos[i] = mJointsPosLimits[0][i];
        }else if(mJointsOutputPos[i] > mJointsPosLimits[1][i]){
            mJointsOutputPos[i] = mJointsPosLimits[1][i];
        }
    }

    switch(mMode){
    case VC_REST:{
        bool bMDone = true;
        if(mPositionController){
            mPositionController->checkMotionDone(&bMDone);
        }
        if(bMDone){
            mMutex.post();
            SetControlMode(VC_IDLE);
            mMutex.wait();
        }
        break;}
    case VC_IDLE:
        if(!bIdleTimeoutPause){
            if(mVelocityController){
                mVelocityController->velocityMove(mJointsOutputVel.data());
            }
        }
        break;
    case VC_POSITION:
        if(mPIDController){
            mPIDController->setReferences(mJointsOutputPos.data());            
        }
        break;
    case VC_COMBINED:
        break;
    case VC_VELOCITY:
        if(mVelocityController)
            mVelocityController->velocityMove(mJointsOutputVel.data());
        break;
    }

    
    mMutex.post();
}

void VelocityController::Stop(){
    if(!bIsReady) return;
    
    mMutex.wait();
    
    if(mEncoders)
        mEncoders->getEncoders(mJointsPos.data());

    mMode = VC_IDLE;
    for(int i=0;i<mJointsSize;i++){
        mJointsTargetPos[i] = mJointsPos[i];
        mJointsTargetVel[i] = 0.0;
    }   
    cout << "Stopping motors: sending zero velocity"<<endl;
    if(mVelocityController){
        mVelocityController->velocityMove(mJointsTargetVel.data());
    }
    if(mControlModeController){
        for(int i=0;i<mJointsSize;i++){
            mControlModeController->setPositionMode(i);
        }
    }
    mMutex.post();
}

char* VelocityController::GetBaseName(){
    return mBaseName;
}

void    VelocityController::GetPosition(Vector &pos){
    pos = mJointsPos;
}
void    VelocityController::GetVelocity(Vector &vel){
    vel = mJointsVel;
}

void    VelocityController::SetPositionTarget(Vector &target){
    mMutex.wait();

    int mx = (target.size()>=mJointsSize?mJointsSize:target.size());
    for(int i=0;i<mx;i++){
        mJointsTargetPos[i] = target[i];   
    }
    for(int i=mx;i<mJointsSize;i++){
        mJointsTargetPos[i] = mJointsPos[i];   
    }
    if(mFirstPos.IsValid()){
        if(mLastPos.IsValid()){
            mPosQueue.push(mLastPos);
        }
        mLastPos.Set(mJointsTargetPos,mTime + POS_OFFSET_TIME, true);
    }else{
        mFirstPos.Set(mJointsTargetPos,mTime +       POS_OFFSET_TIME, true);
        mLastPos.Set( mJointsTargetPos,mTime + 2.0 * POS_OFFSET_TIME, false);
    }



    bPosTargetSet = true;
    
    mMutex.post();
}
void    VelocityController::SetVelocityTarget(Vector &target){
    mMutex.wait();
    
    int mx = (target.size()>=mJointsSize?mJointsSize:target.size());
    for(int i=0;i<mx;i++){
        mJointsTargetVel[i] = target[i];   
    }
    for(int i=mx;i<mJointsSize;i++){
        mJointsTargetVel[i] = 0.0;   
    }
    bVelTargetSet = true;
    
    mMutex.post();
}

void    VelocityController::SetMask(Vector &mask){
    int mx = (mask.size()>=mJointsSize?mJointsSize:mask.size());
    mJointsMask = 0.0;
    for(int i=0;i<mx;i++){
        mJointsMask[i] = (mask[i]>0.0?1.0:0.0);   
    }
}
void    VelocityController::SetMaskAll(){
    for(int i=0;i<mJointsSize;i++){
        mJointsMask[i]=1.0;
    }
}
void    VelocityController::SetMaskNone(){
    for(int i=0;i<mJointsSize;i++){
        mJointsMask[i]=0.0;
    }
}
     
void    VelocityController::SetControlMode(VCMode mode){
    mMutex.wait();
    if(mMode != mode){
        mMode = mode;
        if(mode == VC_IDLE){
            bIdleModeSet = true;
        }else if(mode == VC_REST){
            if(mPositionController){
                mPositionController->positionMove(mJointsRest.data());
            }
        }


        bReset = true;

        switch(mMode){
        case VC_IDLE:
            cout << mBaseName<<"/"<<mName<<": Switching to IDLE mode..."<<endl;
            break;
        case VC_REST:
            cout << mBaseName<<"/"<<mName<<": Switching to REST mode..."<<endl;
            break;
        case VC_POSITION:
            cout << mBaseName<<"/"<<mName<<": Switching to POSITION mode..."<<endl;
            break;
        case VC_VELOCITY:
            cout << mBaseName<<"/"<<mName<<": Switching to VELOCITY mode..."<<endl;
            break;
        case VC_COMBINED:
            cout << mBaseName<<"/"<<mName<<": Switching to COMBINED mode..."<<endl;
            break;
        }
    }
    mMutex.post();
}

void    VelocityController::SetKp(double kps){
    mMutex.wait();
    kps = (kps>0.0?kps:0.0);
    for(int i=0;i<mJointsSize;i++){
        if(mJointsMask[i]!=0.0){
            mJointsKp[i] = kps;
        }   
    }
    mMutex.post();
}
void    VelocityController::SetKd(double kds){
    mMutex.wait();
    kds = (kds>0.0?kds:0.0);
    for(int i=0;i<mJointsSize;i++){
        if(mJointsMask[i]!=0.0){
            mJointsKd[i] = kds;
        }   
    }


    mMutex.post();
}

void    VelocityController::SetCommandTimeout(double timeout){
    mMutex.wait();
    mCommandTimeout = (timeout>0.0?timeout:0.0);
    mMutex.post();
}

void    VelocityController::SetMinimumLoopTime(double time){
    mMutex.wait();
    mMinimumLoopTime = (time>0.0?time:0.0);    
    mMutex.post();
}

int     VelocityController::GetJointsSize(){
    return mJointsSize;    
}

void VelocityController::PQueue(){
    double t0;
    t0 = mFirstPos.mTime;
    if(mFirstPos.IsValid()){
        cout << ""<<mFirstPos.mTime-t0<<" ";
    }else{
        cout << "("<<mFirstPos.mTime-t0<<") ";
    }
    queue<PositionElement> tmp;
    while(!mPosQueue.empty()){
        cout << mPosQueue.front().mTime-t0<<" ";
        tmp.push(mPosQueue.front());
        mPosQueue.pop();
    }
    while(!tmp.empty()){
        mPosQueue.push(tmp.front());
        tmp.pop();
    }

    if(mLastPos.IsValid())
        cout << " "<<mLastPos.mTime-t0<<"  ";
    else
        cout << "<"<<mLastPos.mTime-t0<<"> ";

    cout << endl;
}
